from dataclasses import dataclass
from threading import RLock
from typing import Optional
from enum import Enum

import numpy as np
import rospy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32


class CameraRole(Enum):
    WRIST = "wrist"
    MID = "mid"
    FAR = "far"


@dataclass
class CameraConfig:
    name: str
    label: str
    role: CameraRole
    enabled: bool
    color_topic: str
    color_info_topic: str
    depth_registered_topic: str
    depth_registered_info_topic: str
    lrm_topic: str
    target_frame: str
    prefer_lrm: bool


@dataclass
class CameraCache:
    name: str
    label: str = ""
    color_bgr: Optional[np.ndarray] = None
    color_width: int = 0
    color_height: int = 0
    color_frame_id: str = ""
    color_intrinsics: Optional[np.ndarray] = None

    depth_mm: Optional[np.ndarray] = None
    depth_width: int = 0
    depth_height: int = 0
    depth_frame_id: str = ""
    depth_intrinsics: Optional[np.ndarray] = None

    lrm_mm: int = 0
    last_color_stamp: float = 0.0
    last_depth_stamp: float = 0.0
    last_error: str = ""


class CamerasReader:
    def __init__(self, cameras_configs: dict[str, CameraConfig]):
        self.cameras_configs = cameras_configs
        self.cameras_caches: dict[str, CameraCache] = {
            name: CameraCache(name=config.name, label=config.label)
            for name, config in cameras_configs.items()
        }
        self.lock = RLock()
        self.subscribers = []

    def start(self) -> None:
        for name, config in self.cameras_configs.items():
            self.subscribers.append(
                rospy.Subscriber(
                    config.color_topic,
                    Image,
                    lambda msg, n=name: self._on_color(n, msg),
                    queue_size=1,
                    buff_size=2**24,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    config.color_info_topic,
                    CameraInfo,
                    lambda msg, n=name: self._on_color_info(n, msg),
                    queue_size=1,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    config.depth_registered_topic,
                    Image,
                    lambda msg, n=name: self._on_depth(n, msg),
                    queue_size=1,
                    buff_size=2**24,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    config.depth_registered_info_topic,
                    CameraInfo,
                    lambda msg, n=name: self._on_depth_info(n, msg),
                    queue_size=1,
                )
            )

            if config.prefer_lrm:
                self.subscribers.append(
                    rospy.Subscriber(
                        config.lrm_topic,
                        Float32,
                        lambda msg, n=name: self._on_lrm(n, msg),
                        queue_size=1,
                    )
                )

    def _image_msg_to_numpy(self, msg):
        encoding = (msg.encoding or "").lower()

        encoding_map = {
            "bgr8": (np.uint8, 3),
            "rgb8": (np.uint8, 3),
            "bgra8": (np.uint8, 4),
            "rgba8": (np.uint8, 4),
            "mono8": (np.uint8, 1),
            "8uc1": (np.uint8, 1),
            "mono16": (np.uint16, 1),
            "16uc1": (np.uint16, 1),
            "32fc1": (np.float32, 1),
        }

        if encoding not in encoding_map:
            raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")

        dtype, channels = encoding_map[encoding]
        dtype = np.dtype(dtype)

        data = np.frombuffer(msg.data, dtype=dtype)

        if msg.is_bigendian and data.dtype.byteorder != ">":
            data = data.byteswap().newbyteorder()

        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)

        row_items = step // dtype.itemsize
        image = data.reshape((height, row_items))

        useful_items = width * channels
        image = image[:, :useful_items]

        if channels > 1:
            image = image.reshape((height, width, channels))
        else:
            image = image.reshape((height, width))

        return image

    def _image_msg_to_bgr8(self, msg):
        encoding = (msg.encoding or "").lower()
        image = self._image_msg_to_numpy(msg)

        if encoding == "bgr8":
            return np.ascontiguousarray(image)

        if encoding == "rgb8":
            return np.ascontiguousarray(image[:, :, ::-1])

        if encoding == "bgra8":
            return np.ascontiguousarray(image[:, :, :3])

        if encoding == "rgba8":
            return np.ascontiguousarray(image[:, :, [2, 1, 0]])

        mono = self._mono_to_uint8(image)
        return np.ascontiguousarray(np.repeat(mono[:, :, None], 3, axis=2))

    def _mono_to_uint8(self, image):
        if image.dtype == np.uint8:
            return image

        if np.issubdtype(image.dtype, np.floating):
            finite = image[np.isfinite(image)]
        else:
            finite = image

        if finite.size == 0:
            return np.zeros(image.shape, dtype=np.uint8)

        low = float(np.nanmin(finite))
        high = float(np.nanmax(finite))

        if high <= low:
            return np.zeros(image.shape, dtype=np.uint8)

        scaled = (image.astype(np.float32) - low) * (255.0 / (high - low))
        return (
            np.nan_to_num(scaled, nan=0.0, posinf=255.0, neginf=0.0)
            .clip(0, 255)
            .astype(np.uint8)
        )

    def _on_color(self, name: str, msg: Image) -> None:
        try:
            image = self._image_msg_to_bgr8(msg)

            with self.lock:
                cache = self.cameras_caches[name]
                cache.color_bgr = image
                cache.color_width = msg.width
                cache.color_height = msg.height
                cache.color_frame_id = msg.header.frame_id
                cache.last_color_stamp = msg.header.stamp.to_sec()
                cache.last_error = ""

        except Exception as e:
            with self.lock:
                self.cameras_caches[name].last_error = f"Color image error: {str(e)}"

    def _on_color_info(self, name: str, msg: CameraInfo) -> None:
        try:
            K = np.array(msg.K, dtype=np.float64).reshape(3, 3)

            with self.lock:
                cache = self.cameras_caches[name]
                cache.color_intrinsics = K
                if msg.width > 0:
                    cache.color_width = msg.width
                if msg.height > 0:
                    cache.color_height = msg.height
                    if msg.header.frame_id:
                        cache.color_frame_id = msg.header.frame_id

        except Exception as e:
            with self.lock:
                self.cameras_caches[name].last_error = f"Color info error: {str(e)}"

    def _on_depth(self, name: str, msg: Image) -> None:
        try:
            depth = self._image_msg_to_numpy(msg)

            if msg.encoding == "32FC1":
                depth_mm = depth.astype(np.float32) * 1000.0
            elif msg.encoding in ("16UC1", "mono16"):
                depth_mm = depth.astype(np.float32)
            else:
                raise RuntimeError(f"Unsupported depth encoding: {msg.encoding}")

            with self.lock:
                cache = self.cameras_caches[name]
                cache.depth_mm = depth_mm
                cache.depth_height, cache.depth_width = depth_mm.shape[:2]
                cache.depth_frame_id = msg.header.frame_id
                cache.last_depth_stamp = msg.header.stamp.to_sec()
                cache.last_error = ""

        except Exception as e:
            with self.lock:
                self.cameras_caches[name].last_error = f"Depth image error: {str(e)}"

    def _on_depth_info(self, name: str, msg: CameraInfo) -> None:
        try:
            K = np.array(msg.K, dtype=np.float64).reshape(3, 3)

            with self.lock:
                cache = self.cameras_caches[name]
                cache.depth_intrinsics = K
                if msg.header.frame_id:
                    cache.depth_frame_id = msg.header.frame_id

        except Exception as e:
            with self.lock:
                self.cameras_caches[name].last_error = f"Depth info error: {str(e)}"

    def _on_lrm(self, name: str, msg: Float32) -> None:
        with self.lock:
            self.cameras_caches[name].lrm_mm = int(round(float(msg.data) * 1000.0))

    def snapshot(self, name: str) -> CameraCache:
        with self.lock:
            src = self.cameras_caches[name]
            dst = CameraCache(
                name=src.name,
                label=src.label,
                color_bgr=src.color_bgr.copy() if src.color_bgr is not None else None,
                color_width=src.color_width,
                color_height=src.color_height,
                color_frame_id=src.color_frame_id,
                color_intrinsics=(
                    src.color_intrinsics.copy()
                    if src.color_intrinsics is not None
                    else None
                ),
                depth_mm=src.depth_mm.copy() if src.depth_mm is not None else None,
                depth_width=src.depth_width,
                depth_height=src.depth_height,
                depth_frame_id=src.depth_frame_id,
                depth_intrinsics=(
                    src.depth_intrinsics.copy()
                    if src.depth_intrinsics is not None
                    else None
                ),
                lrm_mm=src.lrm_mm,
                last_color_stamp=src.last_color_stamp,
                last_depth_stamp=src.last_depth_stamp,
                last_error=src.last_error,
            )
            return dst

    def status_dict(self) -> dict:
        with self.lock:
            result = {}
            now = rospy.Time.now().to_sec() if rospy.core.is_initialized() else 0.0
            for name, cache in self.cameras_caches.items():
                result[name] = {
                    "label": cache.label,
                    "color_received": cache.color_bgr is not None,
                    "depth_received": cache.depth_mm is not None,
                    "color_size": (
                        f"{cache.color_width}x{cache.color_height}"
                        if cache.color_bgr is not None
                        else "N/A"
                    ),
                    "depth_size": (
                        f"{cache.depth_width}x{cache.depth_height}"
                        if cache.depth_mm is not None
                        else "N/A"
                    ),
                    "color_frame": cache.color_frame_id,
                    "depth_frame": cache.depth_frame_id,
                    "age_ms": (
                        int((now - cache.last_color_stamp) * 1000.0)
                        if cache.last_color_stamp > 0
                        else -1
                    ),
                    "error": cache.last_error,
                }
            return result
