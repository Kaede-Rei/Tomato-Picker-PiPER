from PySide6.QtCore import QObject, QTimer, Signal, Slot

from typing import Optional
from pathlib import Path

import numpy as np
import rospy
import json
import yaml

from cameras_gui.core.depth import depth_to_pointcloud, get_valid_depth_around
from cameras_gui.core.selection import select_cutting_pixel_from_polygon
from cameras_gui.gui.image_provider import CamerasImageProvider
from cameras_gui.ros.cameras_reader import CameraConfig, CameraRole, CamerasReader
from cameras_gui.ros.pick_action_client import PickActionClient
from cameras_gui.ros.tf_tools import TfTools

import json


def _dumps(payload) -> str:
    return json.dumps(payload, ensure_ascii=False)


def _loads(raw: str) -> dict:
    return json.loads(raw) if raw else {}


def _as_bool(value, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)

    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "on", "enable", "enabled", "启用", "是"}:
        return True
    if text in {"0", "false", "no", "n", "off", "disable", "disabled", "禁用", "否"}:
        return False
    return default


def _as_float(value, default: float = 0.0, name: str = "参数") -> float:
    if value is None or value == "":
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} 必须是数字，当前为 {value!r}")


def _as_int(value, default: int = 0, name: str = "参数", min_value=None, max_value=None) -> int:
    if value is None or value == "":
        result = default
    else:
        try:
            result = int(float(value))
        except (TypeError, ValueError):
            raise ValueError(f"{name} 必须是整数或可识别枚举，当前为 {value!r}")

    if min_value is not None:
        result = max(min_value, result)
    if max_value is not None:
        result = min(max_value, result)
    return result


def _enum_int(value, mapping: dict, default: int, name: str) -> int:
    if value is None or value == "":
        return default

    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return _as_int(value, default=default, name=name)

    text = str(value).strip()
    key = text.upper().replace("-", "_").replace(" ", "_")
    if key in mapping:
        return mapping[key]

    return _as_int(text, default=default, name=name)


def _normalize_task_config(task: dict) -> dict:
    task = task or {}

    task_type = _enum_int(
        task.get("task_type", task.get("taskType", 1)),
        {
            "PICK": 1,
            "TASK_PICK": 1,
            "MOVE_ONLY": 0,
            "MOVE": 0,
            "TASK_MOVE_ONLY": 0,
        },
        default=1,
        name="任务类型",
    )

    group_sort_type = _enum_int(
        task.get("group_sort_type", task.get("group_sort", task.get("groupSort", 0))),
        {
            "ID": 0,
            "GROUP_SORT_ID": 0,
            "DIST": 1,
            "DISTANCE": 1,
            "GROUP_SORT_DIST": 1,
        },
        default=0,
        name="任务组排序方式",
    )

    return {
        "group_name": str(task.get("group_name", task.get("groupName", "gui_pick")) or "gui_pick"),
        "task_id": _as_int(task.get("task_id", task.get("taskId", 1)), default=1, name="任务 ID", min_value=0),
        "description": str(task.get("description", "GUI采摘任务") or "GUI采摘任务"),
        "retry_times": _as_int(task.get("retry_times", task.get("retryTimes", 0)), default=0, name="重试次数", min_value=0, max_value=255),
        "task_type": task_type,
        "group_sort_type": group_sort_type,
        "weight_orient": _as_float(task.get("weight_orient", task.get("weightOrient", 0.30)), default=0.30, name="姿态权重"),
        "use_eef": _as_bool(task.get("use_eef", task.get("useEef", True)), default=True),
        "use_place_pose": _as_bool(task.get("use_place_pose", task.get("use_place", task.get("usePlace", True))), default=True),
        "go_home_after_finish": _as_bool(task.get("go_home_after_finish", task.get("goHomeAfterFinish", True)), default=True),
        "go_safe_after_cancel": _as_bool(task.get("go_safe_after_cancel", task.get("goSafeAfterCancel", True)), default=True),
    }


def _normalize_place_config(place: dict) -> dict:
    place = place or {}
    raw_type = place.get("target_type", place.get("type", "Point"))
    type_key = str(raw_type or "Point").strip().lower().replace("-", "_").replace(" ", "_")
    target_type = "pose" if type_key in {"pose", "place_target_pose"} else "point"

    return {
        "target_type": target_type,
        "frame_id": str(place.get("frame_id", place.get("frame", "base_link")) or "base_link"),
        "x": _as_float(place.get("x", 0.0), default=0.0, name="放置点 X"),
        "y": _as_float(place.get("y", 0.20), default=0.20, name="放置点 Y"),
        "z": _as_float(place.get("z", 0.20), default=0.20, name="放置点 Z"),
        "roll": _as_float(place.get("roll", 0.0), default=0.0, name="放置姿态 Roll"),
        "pitch": _as_float(place.get("pitch", 0.0), default=0.0, name="放置姿态 Pitch"),
        "yaw": _as_float(place.get("yaw", 0.0), default=0.0, name="放置姿态 Yaw"),
    }


def _normalize_tcp_compensation_config(tcp_compensation: dict) -> dict:
    tcp_compensation = tcp_compensation or {}

    return {
        "enabled": _as_bool(tcp_compensation.get("enabled", False), default=False),
        "dx": _as_float(tcp_compensation.get("dx", 0.0), default=0.0, name="TCP 补偿 X"),
        "dy": _as_float(tcp_compensation.get("dy", 0.0), default=0.0, name="TCP 补偿 Y"),
        "dz": _as_float(tcp_compensation.get("dz", 0.0), default=0.0, name="TCP 补偿 Z"),
    }


class CamerasGuiBackend(QObject):
    log_changed = Signal(str)
    state_changed = Signal(str)
    image_changed = Signal(str)
    camera_status_changed = Signal(str)
    preview_status_changed = Signal(str)
    target_changed = Signal(str)
    task_status_changed = Signal(str)

    def __init__(self, image_provider: CamerasImageProvider, config_path: Path):
        super().__init__()
        self.image_provider = image_provider
        self.config_path = config_path
        self.config = self._load_yaml(config_path)

        self.reader: Optional[CamerasReader] = None
        self.tf_tools: Optional[TfTools] = None
        self.pick_client: Optional[PickActionClient] = None

        self.preview_camera = "wrist"
        self.image_revision = 0
        self.last_target: Optional[dict] = None

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)  # ~30 FPS

    def _load_yaml(self, path: Path) -> dict:
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def _parse_camera_role(self, value) -> CameraRole:
        if isinstance(value, CameraRole):
            return value

        role_text = str(value or "wrist").strip().lower()
        for role in CameraRole:
            if role.value == role_text:
                return role

        return CameraRole.WRIST

    @Slot(result=str)
    def state_init(self) -> str:
        cameras = self.config.get("cameras", {})
        camera_names = [name for name, c in cameras.items() if c.get("enabled", True)]
        if camera_names:
            self.preview_camera = camera_names[0]

        return _dumps(
            {
                "config_path": str(self.config_path),
                "camera_names": camera_names,
                "preview_camera": self.preview_camera,
                "cameras": cameras,
                "target_selection": self.config.get("target_selection", {}),
                "task": self.config.get("task", {}),
                "place": self.config.get("place", {}),
                "frames": self.config.get("frames", {}),
                "tcp_compensation": self.config.get("tcp_compensation", {}),
                "task_status": "未连接",
            }
        )

    @Slot(str)
    def connect_ros(self, raw_state: str) -> None:
        state = _loads(raw_state)

        try:
            ros_config = self.config.get("ros", {})
            node_name = ros_config.get("node_name", "tomato_picking_gui_qml")

            if not rospy.core.is_initialized():
                rospy.init_node(node_name, anonymous=True, disable_signals=True)

            camera_configs = {}
            cameras = state.get("cameras", self.config.get("cameras", {}))

            for name, c in cameras.items():
                if not c.get("enabled", True):
                    continue

                camera_configs[name] = CameraConfig(
                    name=name,
                    label=c.get("label", name),
                    role=self._parse_camera_role(c.get("role", name)),
                    enabled=bool(c.get("enabled", True)),
                    color_topic=c.get("color_topic", ""),
                    color_info_topic=c.get("color_info_topic", ""),
                    depth_registered_topic=c.get("depth_registered_topic", ""),
                    depth_registered_info_topic=c.get(
                        "depth_registered_info_topic", ""
                    ),
                    lrm_topic=c.get("lrm_topic", ""),
                    target_frame=c.get("target_frame", "base_link"),
                    prefer_lrm=bool(c.get("prefer_lrm", False)),
                )

            self.reader = CamerasReader(camera_configs)
            self.reader.start()

            tf_wait = float(ros_config.get("tf_wait_sec", 0.2))
            self.tf_tools = TfTools(wait_sec=tf_wait)

            self.pick_client = PickActionClient(
                pick_action_name=ros_config.get("pick_action", "/pick_action"),
                simple_move_action_name=ros_config.get(
                    "simple_move_action", "/simple_move_arm"
                ),
                wait_sec=5.0,
            )
            self.pick_client.set_callbacks(
                feedback_cb=self._on_pick_feedback,
                done_cb=self._on_pick_done,
            )

            self.log_changed.emit("ROS 已连接，相机订阅已启动")
            self.task_status_changed.emit("ROS 已连接")
        except Exception as e:
            self.log_changed.emit(f"ROS 连接失败：{e}")
            self.task_status_changed.emit(f"ROS 连接失败：{e}")

    @Slot(str)
    def set_preview_camera(self, camera_name: str) -> None:
        self.preview_camera = camera_name
        self.preview_status_changed.emit(f"当前相机：{camera_name}")

    def _tick(self) -> None:
        if self.reader is None:
            return

        try:
            snap = self.reader.snapshot(self.preview_camera)
            if snap.color_bgr is not None:
                self.image_provider.set_bgr(self.preview_camera, snap.color_bgr)
                self.image_revision += 1
                self.image_changed.emit(
                    f"image://cameras/{self.preview_camera}?rev={self.image_revision}"
                )

            self.camera_status_changed.emit(_dumps(self.reader.status_dict()))
        except Exception as e:
            self.log_changed.emit(f"刷新相机画面失败：{e}")

    @Slot(str, result=str)
    def commit_polygon_selection(self, raw_payload: str) -> str:
        payload = _loads(raw_payload)
        camera_name = payload.get("camera", self.preview_camera)
        points = payload.get("points", [])

        if self.reader is None or self.tf_tools is None:
            return _dumps({"ok": False, "message": "ROS 未连接"})

        snap = self.reader.snapshot(camera_name)

        if snap.color_bgr is None:
            return _dumps({"ok": False, "message": f"{camera_name} 彩色图未就绪"})

        if snap.depth_mm is None:
            return _dumps({"ok": False, "message": f"{camera_name} 对齐深度图未就绪"})

        if snap.depth_intrinsics is None:
            return _dumps({"ok": False, "message": f"{camera_name} 深度内参未就绪"})

        color_w = snap.color_width
        color_h = snap.color_height
        depth_h, depth_w = snap.depth_mm.shape[:2]

        if (depth_w, depth_h) != (color_w, color_h):
            return _dumps(
                {
                    "ok": False,
                    "message": (
                        f"{camera_name} 对齐深度图尺寸 {depth_w}x{depth_h} "
                        f"与彩色图尺寸 {color_w}x{color_h} 不一致，不能按 RGB 像素取深度"
                    ),
                }
            )

        sel_config = self.config.get("target_selection", {})
        point_method = payload.get(
            "pointMethod", sel_config.get("point_method", "skeleton_centroid")
        )

        cutting_pixel, mask = select_cutting_pixel_from_polygon(
            width=color_w,
            height=color_h,
            polygon_points=points,
            method=point_method,
            close_kernel=int(sel_config.get("mask_close_kernel", 5)),
            open_kernel=int(sel_config.get("mask_open_kernel", 3)),
        )

        u = int(np.clip(round(cutting_pixel[0]), 0, depth_w - 1))
        v = int(np.clip(round(cutting_pixel[1]), 0, depth_h - 1))

        depth_mm = get_valid_depth_around(
            depth_mm=snap.depth_mm,
            u=u,
            v=v,
            mask=mask,
            kernel_size=int(sel_config.get("depth_kernel_size", 7)),
            trim_ratio=float(sel_config.get("depth_trim_ratio", 0.15)),
            close_kernel=int(sel_config.get("mask_close_kernel", 5)),
            open_kernel=int(sel_config.get("mask_open_kernel", 3)),
            center_band_min_dist_px=float(
                sel_config.get("center_band_min_dist_px", 1.0)
            ),
        )

        camera_config = self.reader.cameras_configs[camera_name]
        use_lrm = False
        if camera_config.prefer_lrm and 1 <= snap.lrm_mm <= 400:
            depth_mm = float(snap.lrm_mm)
            use_lrm = True

        if depth_mm <= 0:
            return _dumps({"ok": False, "message": "深度无效，请重新框选"})

        depth_m = depth_mm / 1000.0
        cam_xyz = depth_to_pointcloud(u, v, depth_m, snap.depth_intrinsics)

        output_frame = payload.get("outputFrame") or camera_config.target_frame
        target_xyz = self.tf_tools.transform_point(
            snap.depth_frame_id,
            output_frame,
            cam_xyz,
        )

        raw_target_xyz = target_xyz
        tcp_comp = _normalize_tcp_compensation_config(
            payload.get(
                "tcpCompensation",
                payload.get("tcp_compensation", self.config.get("tcp_compensation", {})),
            )
        )
        tcp_frame = self.config.get("frames", {}).get("tcp_frame", "link_tcp")
        tcp_comp_applied = False
        tcp_comp_skipped_reason = ""

        if tcp_comp["enabled"]:
            if output_frame == tcp_frame:
                target_xyz = (
                    target_xyz[0] + tcp_comp["dx"],
                    target_xyz[1] + tcp_comp["dy"],
                    target_xyz[2] + tcp_comp["dz"],
                )
                tcp_comp_applied = True
            else:
                tcp_comp_skipped_reason = (
                    f"输出坐标系为 {output_frame}，不是 TCP 坐标系 {tcp_frame}"
                )

        result = {
            "ok": True,
            "camera": camera_name,
            "pixel": {"u": float(cutting_pixel[0]), "v": float(cutting_pixel[1])},
            "depthM": depth_m,
            "useLrm": use_lrm,
            "cameraFrame": snap.depth_frame_id,
            "cameraXYZ": list(cam_xyz),
            "targetFrame": output_frame,
            "rawTargetXYZ": list(raw_target_xyz),
            "targetXYZ": list(target_xyz),
            "tcpCompensation": {
                **tcp_comp,
                "applied": tcp_comp_applied,
                "skippedReason": tcp_comp_skipped_reason,
            },
        }

        self.last_target = result
        self.target_changed.emit(_dumps(result))
        return _dumps(result)

    @Slot(str, result=str)
    def upsert_current_task(self, raw_state: str) -> str:
        if self.pick_client is None:
            return _dumps({"ok": False, "message": "采摘动作未连接"})

        if not self.last_target or not self.last_target.get("ok"):
            return _dumps({"ok": False, "message": "尚未选择有效目标"})

        state = _loads(raw_state)

        try:
            task = _normalize_task_config(state.get("task", self.config.get("task", {})))
            place = _normalize_place_config(state.get("place", self.config.get("place", {})))
            group_config = {
                "group_sort_type": task["group_sort_type"],
                "weight_orient": task["weight_orient"],
                "go_home_after_finish": task["go_home_after_finish"],
            }

            self.pick_client.upsert_task(
                group_name=task["group_name"],
                task_id=task["task_id"],
                description=task["description"],
                target_xyz=tuple(self.last_target["targetXYZ"]),
                target_frame_id=self.last_target["targetFrame"],
                task_type=task["task_type"],
                use_eef=task["use_eef"],
                retry_times=task["retry_times"],
                go_safe_after_cancel=task["go_safe_after_cancel"],
                use_place_pose=task["use_place_pose"],
                place_config=place,
                group_config=group_config,
            )
            msg = "任务已写入 / 更新"
            self.task_status_changed.emit(msg)
            return _dumps({"ok": True, "message": msg})
        except Exception as e:
            return _dumps({"ok": False, "message": str(e)})

    @Slot(str, result=str)
    def execute_group(self, raw_state: str) -> str:
        if self.pick_client is None:
            return _dumps({"ok": False, "message": "采摘动作未连接"})

        state = _loads(raw_state)

        try:
            task = _normalize_task_config(state.get("task", self.config.get("task", {})))
            group_config = {
                "group_sort_type": task["group_sort_type"],
                "weight_orient": task["weight_orient"],
                "go_home_after_finish": task["go_home_after_finish"],
            }

            self.pick_client.execute_group(
                group_name=task["group_name"],
                use_eef=task["use_eef"],
                retry_times=task["retry_times"],
                go_safe_after_cancel=task["go_safe_after_cancel"],
                group_config=group_config,
            )
            msg = "任务组执行请求已发送"
            self.task_status_changed.emit(msg)
            return _dumps({"ok": True, "message": msg})
        except Exception as e:
            return _dumps({"ok": False, "message": str(e)})

    @Slot()
    def cancel_task(self) -> None:
        if self.pick_client:
            self.pick_client.cancel()
            self.task_status_changed.emit("已请求取消当前任务")

    @Slot()
    def go_home(self) -> None:
        if not self.pick_client:
            self.task_status_changed.emit("未连接 /simple_move_arm")
            return
        try:
            self.pick_client.go_home(done_cb=self._on_go_home_done)
            self.task_status_changed.emit("正在返回安全区")
        except Exception as e:
            self.task_status_changed.emit(f"返回安全区失败：{e}")

    def _on_pick_feedback(self, feedback) -> None:
        try:
            self.task_status_changed.emit(
                f"{feedback.stage_text} | 步骤 {feedback.current_step_index}/{feedback.total_steps}"
            )
        except Exception:
            pass

    def _on_pick_done(self, state, result) -> None:
        try:
            msg = getattr(result, "message", "")
            success = bool(getattr(result, "success", False))
            if success:
                self.task_status_changed.emit(f"任务完成：{msg}")
            elif bool(getattr(result, "canceled", False)):
                self.task_status_changed.emit(f"任务已取消：{msg}")
            else:
                self.task_status_changed.emit(f"任务失败：{msg}")
        except Exception:
            self.task_status_changed.emit(f"任务结束，状态={state}")

    def _on_go_home_done(self, state, result) -> None:
        try:
            if bool(getattr(result, "success", False)):
                self.task_status_changed.emit("返回安全区成功")
            else:
                self.task_status_changed.emit(
                    f"返回安全区失败：{getattr(result, 'message', '')}"
                )
        except Exception:
            self.task_status_changed.emit(f"返回安全区结束，状态={state}")
