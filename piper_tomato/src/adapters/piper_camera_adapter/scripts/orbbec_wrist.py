#!/usr/bin/env python3
import os
import re
from typing import Any, List, Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

from pyorbbecsdk import (
    OBAlignMode,
    OBFormat,
    OBPropertyID,
    OBSensorType,
    Config,
    Context,
    Pipeline,
    VideoFrame,
)

try:
    from pyorbbecsdk import AlignFilter, OBFrameAggregateOutputMode, OBStreamType
except ImportError:
    AlignFilter = None
    OBFrameAggregateOutputMode = None
    OBStreamType = None


DEFAULT_ZERO_DISTORTION = [0.0, 0.0, 0.0, 0.0, 0.0]


def intrinsic_to_matrix(intrinsic: Any) -> np.ndarray:
    return np.array(
        [
            [float(intrinsic.fx), 0.0, float(intrinsic.cx)],
            [0.0, float(intrinsic.fy), float(intrinsic.cy)],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def frame_to_bgr_image(frame: VideoFrame) -> Optional[np.ndarray]:
    width = frame.get_width()
    height = frame.get_height()
    color_format = frame.get_format()
    data = np.asanyarray(frame.get_data())

    if color_format == OBFormat.RGB:
        image = np.frombuffer(frame.get_data(), dtype=np.uint8).reshape(
            height, width, 3
        )
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if color_format == OBFormat.YUYV:
        image = np.frombuffer(frame.get_data(), dtype=np.uint8).reshape(
            height, width, 2
        )
        return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)

    if color_format == OBFormat.MJPG:
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    rospy.logwarn_throttle(2.0, "Unsupported color format: %s", color_format)
    return None


def rotate_intrinsics_180(K: np.ndarray, width: int, height: int) -> np.ndarray:
    rotated = np.array(K, dtype=np.float64, copy=True)
    rotated[0, 2] = float(width - 1) - float(K[0, 2])
    rotated[1, 2] = float(height - 1) - float(K[1, 2])
    return rotated


def _usb_uid_from_video_device(video_device: str) -> str:
    video_name = os.path.basename(os.path.realpath(video_device))
    if not video_name.startswith("video"):
        return ""

    sys_device = f"/sys/class/video4linux/{video_name}/device"
    try:
        resolved = os.path.realpath(sys_device)
    except OSError:
        return ""

    matches = re.findall(r"([0-9]+-[0-9]+(?:\.[0-9]+)*)(?::[0-9.]+)?", resolved)
    return matches[-1] if matches else ""


def _usb_uid_from_com_alias(alias_name: str) -> str:
    match = re.fullmatch(r"com-([0-9][0-9.-]*)-video", alias_name)
    if not match:
        return ""

    token = match.group(1)
    if "-" not in token:
        token = f"1-{token}"

    sys_usb_devices = "/sys/bus/usb/devices"
    try:
        for device_name in os.listdir(sys_usb_devices):
            if ":" in device_name or not device_name.endswith(token):
                continue
            vendor_path = os.path.join(sys_usb_devices, device_name, "idVendor")
            try:
                with open(vendor_path, "r", encoding="utf-8") as vendor_file:
                    if vendor_file.read().strip() == "2bc5":
                        return device_name
            except OSError:
                pass
    except OSError:
        pass

    return token


def resolve_usb_port_selector(selector: str) -> str:
    value = str(selector or "").strip()
    if not value:
        return ""

    video_match = re.fullmatch(r"(?:/dev/)?video([0-9]+)", value)
    if video_match:
        uid = _usb_uid_from_video_device(f"/dev/video{video_match.group(1)}")
        return uid or value

    if re.fullmatch(r"[0-9]+", value):
        uid = _usb_uid_from_video_device(f"/dev/video{value}")
        return uid or value

    if value.startswith("/dev/"):
        uid = _usb_uid_from_video_device(value)
        if uid:
            return uid
        alias_uid = _usb_uid_from_com_alias(os.path.basename(value))
        if alias_uid:
            return alias_uid

    return value


class OrbbecRgbdNode:
    def __init__(self) -> None:
        rospy.init_node("orbbec_rgbd_node")
        self.bridge = CvBridge()

        self.camera_role = rospy.get_param("~camera_role", "wrist")
        self.output_ns = self._normalize_topic_ns(
            rospy.get_param("~output_ns", f"/camera/{self.camera_role}")
        )
        self.usb_port = rospy.get_param("~usb_port", "")
        self.camera_index = rospy.get_param("~camera_index", "")
        self.serial_number = rospy.get_param("~serial_number", "")

        self.color_width = int(rospy.get_param("~color_width", 1280))
        self.color_height = int(rospy.get_param("~color_height", 720))
        self.color_fps = int(rospy.get_param("~color_fps", 30))
        self.depth_width = int(rospy.get_param("~depth_width", 1280))
        self.depth_height = int(rospy.get_param("~depth_height", 800))
        self.depth_fps = int(rospy.get_param("~depth_fps", 30))

        self.align_to_color = bool(rospy.get_param("~align_to_color", True))
        self.rotate_180 = bool(rospy.get_param("~rotate_180", False))
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.10))
        self.max_depth_m = float(rospy.get_param("~max_depth_m", 10.0))
        self.enable_lrm = bool(rospy.get_param("~enable_lrm", True))
        self.publish_rate = float(
            rospy.get_param("~publish_rate", max(1.0, self.color_fps))
        )

        default_prefix = f"{self.camera_role}_cam"
        self.color_frame_id = rospy.get_param(
            "~color_frame_id", f"{default_prefix}_color_optical_frame"
        )
        self.depth_frame_id = rospy.get_param(
            "~depth_frame_id", f"{default_prefix}_depth_optical_frame"
        )
        self.depth_registered_frame_id = rospy.get_param(
            "~depth_registered_frame_id", self.color_frame_id
        )

        self.distortion_model = rospy.get_param("~distortion_model", "plumb_bob")
        self.color_distortion_override = self._normalize_distortion_param(
            rospy.get_param("~color_distortion", DEFAULT_ZERO_DISTORTION)
        )
        self.depth_distortion_override = self._normalize_distortion_param(
            rospy.get_param("~depth_distortion", DEFAULT_ZERO_DISTORTION)
        )

        self.pipeline: Optional[Pipeline] = None
        self.device = None
        self.align_filter = None

        self.color_pub = rospy.Publisher(
            self._topic("color/image_raw"), Image, queue_size=1
        )
        self.color_info_pub = rospy.Publisher(
            self._topic("color/camera_info"), CameraInfo, queue_size=1
        )
        self.depth_raw_pub = rospy.Publisher(
            self._topic("depth/image_raw"), Image, queue_size=1
        )
        self.depth_raw_info_pub = rospy.Publisher(
            self._topic("depth/camera_info"), CameraInfo, queue_size=1
        )
        self.depth_registered_pub = rospy.Publisher(
            self._topic("depth_registered/image_raw"), Image, queue_size=1
        )
        self.depth_registered_info_pub = rospy.Publisher(
            self._topic("depth_registered/camera_info"), CameraInfo, queue_size=1
        )
        self.lrm_pub = rospy.Publisher(
            self._topic("lrm_distance"), Float32, queue_size=1
        )

    @staticmethod
    def _normalize_topic_ns(raw: str) -> str:
        ns = str(raw or "/camera/wrist").strip()
        if not ns.startswith("/"):
            ns = "/" + ns
        return ns.rstrip("/")

    def _topic(self, suffix: str) -> str:
        return f"{self.output_ns}/{suffix.lstrip('/')}"

    @staticmethod
    def _normalize_distortion_param(raw: Any) -> List[float]:
        if raw is None:
            return list(DEFAULT_ZERO_DISTORTION)
        if isinstance(raw, (int, float)):
            return [float(raw)]
        if isinstance(raw, (list, tuple)):
            return [float(v) for v in raw]
        return list(DEFAULT_ZERO_DISTORTION)

    def _select_color_profile(self, profile_list):
        try:
            return profile_list.get_video_stream_profile(
                self.color_width, self.color_height, OBFormat.RGB, self.color_fps
            )
        except Exception:
            try:
                return profile_list.get_video_stream_profile(
                    self.color_width, self.color_height, OBFormat.MJPG, self.color_fps
                )
            except Exception:
                return profile_list.get_default_video_stream_profile()

    def _select_depth_profile(self, profile_list):
        try:
            return profile_list.get_video_stream_profile(
                self.depth_width, self.depth_height, OBFormat.Y16, self.depth_fps
            )
        except Exception:
            return profile_list.get_default_video_stream_profile()

    def _set_disparity_search_range_256(self) -> bool:
        if self.device is None:
            return False

        prop = getattr(OBPropertyID, "OB_PROP_DISP_SEARCH_RANGE_MODE_INT", None)
        if prop is None:
            rospy.logwarn(
                "pyorbbecsdk does not expose OB_PROP_DISP_SEARCH_RANGE_MODE_INT"
            )
            return False

        try:
            self.device.set_int_property(prop, 2)
            applied = self.device.get_int_property(prop)
            rospy.loginfo("Disparity search range mode: %s", applied)
            return applied == 2
        except Exception as exc:
            rospy.logwarn("Failed to set disparity search range: %s", exc)
            return False

    def _build_depth_m(self, depth_frame: VideoFrame) -> np.ndarray:
        depth_u16 = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(
            (depth_frame.get_height(), depth_frame.get_width())
        )
        depth_u16 = cv2.medianBlur(depth_u16, 5)
        depth_m = (
            depth_u16.astype(np.float32) * float(depth_frame.get_depth_scale()) / 1000.0
        )
        depth_m = np.where(
            (depth_m > self.min_depth_m) & (depth_m < self.max_depth_m), depth_m, 0.0
        ).astype(np.float32)
        return depth_m

    def _extract_distortion_from_profile(self, profile: Any) -> Optional[List[float]]:
        candidates = []
        for getter_name in (
            "get_distortion",
            "get_distortion_param",
            "get_distortion_params",
            "get_camera_param",
            "get_camera_intrinsic",
        ):
            getter = getattr(profile, getter_name, None)
            if callable(getter):
                try:
                    candidates.append(getter())
                except Exception:
                    pass

        for candidate in candidates:
            parsed = self._parse_distortion_object(candidate)
            if parsed is not None:
                return parsed
        return None

    def _parse_distortion_object(self, obj: Any) -> Optional[List[float]]:
        if obj is None:
            return None
        if isinstance(obj, (list, tuple, np.ndarray)):
            values = [float(v) for v in obj]
            if len(values) >= 4:
                return values
        if hasattr(obj, "coeffs"):
            return self._parse_distortion_object(getattr(obj, "coeffs"))

        fields = []
        for name in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"):
            if hasattr(obj, name):
                fields.append(float(getattr(obj, name)))
        if len(fields) >= 4:
            return fields

        for name in (
            "color_distortion",
            "depth_distortion",
            "rgb_distortion",
            "distortion",
        ):
            if hasattr(obj, name):
                parsed = self._parse_distortion_object(getattr(obj, name))
                if parsed is not None:
                    return parsed
        return None

    def _resolve_distortion(self, profile: Any, override: List[float]) -> List[float]:
        extracted = self._extract_distortion_from_profile(profile)
        return extracted if extracted is not None else list(override)

    def _build_camera_info(
        self,
        width: int,
        height: int,
        K: np.ndarray,
        D: List[float],
        frame_id: str,
        stamp: rospy.Time,
    ) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.width = int(width)
        msg.height = int(height)
        msg.distortion_model = self.distortion_model
        msg.D = [float(v) for v in D]
        msg.K = [
            float(K[0, 0]),
            0.0,
            float(K[0, 2]),
            0.0,
            float(K[1, 1]),
            float(K[1, 2]),
            0.0,
            0.0,
            1.0,
        ]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [
            float(K[0, 0]),
            0.0,
            float(K[0, 2]),
            0.0,
            0.0,
            float(K[1, 1]),
            float(K[1, 2]),
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return msg

    def _to_msg(
        self, image: np.ndarray, encoding: str, stamp: rospy.Time, frame_id: str
    ) -> Image:
        msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        return msg

    def setup(self) -> None:
        self.pipeline = self._create_pipeline()
        self.device = self.pipeline.get_device()
        config = Config()

        color_profile_list = self.pipeline.get_stream_profile_list(
            OBSensorType.COLOR_SENSOR
        )
        depth_profile_list = self.pipeline.get_stream_profile_list(
            OBSensorType.DEPTH_SENSOR
        )
        color_profile = self._select_color_profile(color_profile_list)
        depth_profile = self._select_depth_profile(depth_profile_list)

        config.enable_stream(color_profile)
        config.enable_stream(depth_profile)

        if self.align_to_color and hasattr(config, "set_align_mode"):
            try:
                config.set_align_mode(OBAlignMode.SW_MODE)
            except Exception:
                pass

        if (
            hasattr(config, "set_frame_aggregate_output_mode")
            and OBFrameAggregateOutputMode is not None
            and hasattr(OBFrameAggregateOutputMode, "FULL_FRAME_REQUIRE")
        ):
            try:
                config.set_frame_aggregate_output_mode(
                    OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE
                )
            except Exception:
                pass

        self.pipeline.enable_frame_sync()
        self.pipeline.start(config)

        if self.align_to_color:
            self._setup_align_filter()

        disparity_ok = self._set_disparity_search_range_256()

        if self.enable_lrm:
            try:
                self.device.set_bool_property(OBPropertyID.OB_PROP_LDP_BOOL, True)
                rospy.loginfo("LRM module enabled")
            except Exception as exc:
                rospy.logwarn("Failed to enable LRM module: %s", exc)

        rospy.loginfo(
            "Camera ready: role=%s ns=%s selector=%s serial=%s disp256=%s rotate_180=%s color_frame=%s depth_frame=%s registered_frame=%s",
            self.camera_role,
            self.output_ns,
            self.usb_port or self.camera_index or "<auto>",
            self.serial_number or "<auto>",
            "OK" if disparity_ok else "NO",
            self.rotate_180,
            self.color_frame_id,
            self.depth_frame_id,
            self.depth_registered_frame_id,
        )

    def _setup_align_filter(self) -> None:
        if AlignFilter is None or OBStreamType is None:
            rospy.logwarn(
                "pyorbbecsdk does not expose AlignFilter; depth_registered will not be published"
            )
            self.align_filter = None
            return

        try:
            self.align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
            rospy.loginfo("AlignFilter enabled: depth -> color")
        except Exception as exc:
            self.align_filter = None
            rospy.logwarn("Failed to initialize AlignFilter: %s", exc)

    def _create_pipeline(self) -> Pipeline:
        selector = self.usb_port or self.camera_index
        if self.usb_port and self.camera_index:
            raise RuntimeError("usb_port and camera_index are mutually exclusive")
        if self.serial_number and selector:
            raise RuntimeError(
                "serial_number and usb_port/camera_index are mutually exclusive"
            )

        if not self.serial_number and not selector:
            rospy.logwarn(
                "No device selector specified for %s; using pyorbbecsdk default device",
                self.camera_role,
            )
            return Pipeline()

        context = Context()
        device_list = context.query_devices()

        if self.serial_number:
            rospy.loginfo(
                "Selecting %s by serial_number: %s",
                self.camera_role,
                self.serial_number,
            )
            device = device_list.get_device_by_serial_number(self.serial_number)
        else:
            resolved_selector = resolve_usb_port_selector(selector)
            rospy.loginfo(
                "Selecting %s by usb/index: %s -> %s",
                self.camera_role,
                selector,
                resolved_selector,
            )
            device = device_list.get_device_by_uid(resolved_selector)

        if device is None:
            raise RuntimeError(
                f"Cannot find {self.camera_role} camera: "
                f"serial_number={self.serial_number or '<empty>'}, "
                f"usb_port={self.usb_port or '<empty>'}, "
                f"camera_index={self.camera_index or '<empty>'}"
            )

        try:
            info = device.get_device_info()
            rospy.loginfo(
                "Selected %s camera: name=%s serial=%s uid=%s",
                self.camera_role,
                info.get_name(),
                info.get_serial_number(),
                info.get_uid(),
            )
        except Exception as exc:
            rospy.logwarn("Failed to read %s device info: %s", self.camera_role, exc)

        return Pipeline(device)

    def spin(self) -> None:
        assert self.pipeline is not None
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames(100)
            if frames is None:
                rate.sleep()
                continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if color_frame is None or depth_frame is None:
                rate.sleep()
                continue

            aligned_depth_frame = self._try_get_aligned_depth_frame(frames)
            color_image = frame_to_bgr_image(color_frame)
            if color_image is None:
                rate.sleep()
                continue

            raw_depth_m = self._build_depth_m(depth_frame)
            aligned_depth_m = self._try_build_aligned_depth_m(aligned_depth_frame)

            color_profile = color_frame.get_stream_profile().as_video_stream_profile()
            depth_profile = depth_frame.get_stream_profile().as_video_stream_profile()
            K_color = intrinsic_to_matrix(color_profile.get_intrinsic())
            K_depth = intrinsic_to_matrix(depth_profile.get_intrinsic())
            D_color = self._resolve_distortion(
                color_profile, self.color_distortion_override
            )
            D_depth = self._resolve_distortion(
                depth_profile, self.depth_distortion_override
            )

            if self.rotate_180:
                color_image, raw_depth_m, aligned_depth_m, K_color, K_depth = (
                    self._rotate_outputs(
                        color_image, raw_depth_m, aligned_depth_m, K_color, K_depth
                    )
                )

            now = rospy.Time.now()
            self._publish_color(color_image, K_color, D_color, now)
            self._publish_depth(raw_depth_m, K_depth, D_depth, now)
            self._publish_registered_depth_if_valid(
                aligned_depth_m, color_image, K_color, D_color, now
            )
            self._publish_lrm_distance()

            rate.sleep()

    def _try_get_aligned_depth_frame(self, frames) -> Optional[VideoFrame]:
        if self.align_filter is None:
            return None
        try:
            aligned_frames = self.align_filter.process(frames)
            if aligned_frames is None:
                return None
            return aligned_frames.as_frame_set().get_depth_frame()
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "AlignFilter processing failed: %s", exc)
            return None

    def _try_build_aligned_depth_m(
        self, aligned_depth_frame: Optional[VideoFrame]
    ) -> Optional[np.ndarray]:
        if aligned_depth_frame is None:
            return None
        try:
            return self._build_depth_m(aligned_depth_frame)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse aligned depth: %s", exc)
            return None

    def _rotate_outputs(
        self,
        color_image: np.ndarray,
        raw_depth_m: np.ndarray,
        aligned_depth_m: Optional[np.ndarray],
        K_color: np.ndarray,
        K_depth: np.ndarray,
    ):
        color_image = cv2.rotate(color_image, cv2.ROTATE_180)
        raw_depth_m = cv2.rotate(raw_depth_m, cv2.ROTATE_180)
        K_color = rotate_intrinsics_180(
            K_color, color_image.shape[1], color_image.shape[0]
        )
        K_depth = rotate_intrinsics_180(
            K_depth, raw_depth_m.shape[1], raw_depth_m.shape[0]
        )
        if aligned_depth_m is not None:
            aligned_depth_m = cv2.rotate(aligned_depth_m, cv2.ROTATE_180)
        return color_image, raw_depth_m, aligned_depth_m, K_color, K_depth

    def _publish_color(
        self, image: np.ndarray, K: np.ndarray, D: List[float], stamp: rospy.Time
    ) -> None:
        self.color_pub.publish(self._to_msg(image, "bgr8", stamp, self.color_frame_id))
        self.color_info_pub.publish(
            self._build_camera_info(
                image.shape[1], image.shape[0], K, D, self.color_frame_id, stamp
            )
        )

    def _publish_depth(
        self, depth_m: np.ndarray, K: np.ndarray, D: List[float], stamp: rospy.Time
    ) -> None:
        self.depth_raw_pub.publish(
            self._to_msg(depth_m, "32FC1", stamp, self.depth_frame_id)
        )
        self.depth_raw_info_pub.publish(
            self._build_camera_info(
                depth_m.shape[1], depth_m.shape[0], K, D, self.depth_frame_id, stamp
            )
        )

    def _publish_registered_depth_if_valid(
        self,
        aligned_depth_m: Optional[np.ndarray],
        color_image: np.ndarray,
        K_color: np.ndarray,
        D_color: List[float],
        stamp: rospy.Time,
    ) -> None:
        if aligned_depth_m is None:
            return
        if aligned_depth_m.shape[:2] != color_image.shape[:2]:
            rospy.logwarn_throttle(
                2.0,
                "Skip depth_registered because aligned depth size %s != color size %s",
                aligned_depth_m.shape[:2],
                color_image.shape[:2],
            )
            return

        self.depth_registered_pub.publish(
            self._to_msg(
                aligned_depth_m, "32FC1", stamp, self.depth_registered_frame_id
            )
        )
        self.depth_registered_info_pub.publish(
            self._build_camera_info(
                aligned_depth_m.shape[1],
                aligned_depth_m.shape[0],
                K_color,
                D_color,
                self.depth_registered_frame_id,
                stamp,
            )
        )

    def _publish_lrm_distance(self) -> None:
        if not self.enable_lrm or self.device is None:
            return
        try:
            lrm_dist_mm = self.device.get_int_property(
                OBPropertyID.OB_PROP_LDP_MEASURE_DISTANCE_INT
            )
            self.lrm_pub.publish(Float32(data=float(lrm_dist_mm) / 1000.0))
        except Exception:
            pass


if __name__ == "__main__":
    node = OrbbecRgbdNode()
    node.setup()
    node.spin()
