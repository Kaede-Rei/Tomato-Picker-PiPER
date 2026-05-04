from typing import Optional
from pathlib import Path

import numpy as np
import rospy
import json
import yaml

from PySide6.QtCore import QObject, QTimer, Signal, Slot
from cameras_gui.core.depth import depth_to_pointcloud, get_valid_depth_around
from cameras_gui.core.selection import select_cutting_pixel_from_polygon
from cameras_gui.gui.image_provider import CamerasImageProvider
from cameras_gui.ros.cameras_reader import CameraConfig, CamerasReader
from cameras_gui.ros.pick_action_client import PickActionClient
from cameras_gui.ros.tf_tools import TfTools

import json


def _dumps(payload) -> str:
    return json.dumps(payload, ensure_ascii=False)


def _loads(raw: str) -> dict:
    return json.loads(raw) if raw else {}


class PiperGuiBackend(QObject):
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

    @Slot(result=str)
    def state_init(self) -> str:
        cameras = self.config.get("cameras", {})
        camera_names = [name for name, c in cameras.item() if c.get("enabled", True)]
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
                    color_topic=c.get("color_topic", ""),
                    color_info_topic=c.get("color_info_topic", ""),
                    depth_registered_topic=c.get("depth_registered_topic", ""),
                    depth_registered_info_topic=c.get(
                        "depth_registered_info_topic", ""
                    ),
                    lrm_topic=c.get("lrm_topic", ""),
                    default_output_frame=c.get("default_output_frame", "base_link"),
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

            self.logChanged.emit("ROS 已连接，相机订阅已启动")
            self.taskStatusChanged.emit("ROS 已连接")
        except Exception as e:
            self.logChanged.emit(f"ROS 连接失败：{e}")
            self.taskStatusChanged.emit(f"ROS 连接失败：{e}")

    @Slot(str)
    def set_preview_camera(self, camera_name: str) -> None:
        self.preview_camera = camera_name
        self.previewStatusChanged.emit(f"当前相机：{camera_name}")

    def _tick(self) -> None:
        if self.reader is None:
            return

        try:
            snap = self.reader.snapshot(self.preview_camera)
            if snap.color_bgr is not None:
                self.image_provider.set_bgr(self.preview_camera, snap.color_bgr)
                self.image_revision += 1
                self.imageChanged.emit(
                    f"image://piper_camera/{self.preview_camera}?rev={self.image_revision}"
                )

            self.cameraStatusChanged.emit(_dumps(self.reader.status_dict()))
        except Exception as e:
            self.logChanged.emit(f"刷新相机画面失败：{e}")

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

        camera_config = self.reader.camera_configs[camera_name]
        use_lrm = False
        if camera_config.prefer_lrm and 1 <= snap.lrm_mm <= 400:
            depth_mm = float(snap.lrm_mm)
            use_lrm = True

        if depth_mm <= 0:
            return _dumps({"ok": False, "message": "深度无效，请重新框选"})

        depth_m = depth_mm / 1000.0
        cam_xyz = depth_to_pointcloud(u, v, depth_m, snap.depth_intrinsics)

        output_frame = payload.get("outputFrame") or camera_config.default_output_frame
        target_xyz = self.tf_tools.transform_point(
            snap.depth_frame_id,
            output_frame,
            cam_xyz,
        )

        tcp_comp = self.config.get("tcp_compensation", {})
        if output_frame == self.config.get("frames", {}).get("tcp_frame", "link_tcp"):
            if bool(tcp_comp.get("enabled", False)):
                target_xyz = (
                    target_xyz[0] + float(tcp_comp.get("dx", 0.0)),
                    target_xyz[1] + float(tcp_comp.get("dy", 0.0)),
                    target_xyz[2] + float(tcp_comp.get("dz", 0.0)),
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
            "targetXYZ": list(target_xyz),
        }

        self.last_target = result
        self.targetChanged.emit(_dumps(result))
        return _dumps(result)

    @Slot(str, result=str)
    def upsert_current_task(self, raw_state: str) -> str:
        if self.pick_client is None:
            return _dumps({"ok": False, "message": "采摘动作未连接"})

        if not self.last_target or not self.last_target.get("ok"):
            return _dumps({"ok": False, "message": "尚未选择有效目标"})

        state = _loads(raw_state)
        task = state.get("task", self.config.get("task", {}))
        place = state.get("place", self.config.get("place", {}))
        group_config = {
            "group_sort_type": int(task.get("group_sort_type", 0)),
            "weight_orient": float(task.get("weight_orient", 0.30)),
            "go_home_after_finish": bool(task.get("go_home_after_finish", True)),
        }

        try:
            self.pick_client.upsert_task(
                group_name=task.get("group_name", "gui_pick"),
                task_id=int(task.get("task_id", 1)),
                description=task.get("description", "GUI采摘任务"),
                target_xyz=tuple(self.last_target["targetXYZ"]),
                target_frame_id=self.last_target["targetFrame"],
                task_type=int(task.get("task_type", 0)),
                use_eef=bool(task.get("use_eef", True)),
                retry_times=int(task.get("retry_times", 0)),
                go_safe_after_cancel=bool(task.get("go_safe_after_cancel", True)),
                use_place_pose=bool(task.get("use_place_pose", True)),
                place_config=place,
                group_config=group_config,
            )
            msg = "任务已写入 / 更新"
            self.taskStatusChanged.emit(msg)
            return _dumps({"ok": True, "message": msg})
        except Exception as e:
            return _dumps({"ok": False, "message": str(e)})

    @Slot(str, result=str)
    def execute_group(self, raw_state: str) -> str:
        if self.pick_client is None:
            return _dumps({"ok": False, "message": "采摘动作未连接"})

        state = _loads(raw_state)
        task = state.get("task", self.config.get("task", {}))
        group_config = {
            "group_sort_type": int(task.get("group_sort_type", 0)),
            "weight_orient": float(task.get("weight_orient", 0.30)),
            "go_home_after_finish": bool(task.get("go_home_after_finish", True)),
        }

        try:
            self.pick_client.execute_group(
                group_name=task.get("group_name", "gui_pick"),
                use_eef=bool(task.get("use_eef", True)),
                retry_times=int(task.get("retry_times", 0)),
                go_safe_after_cancel=bool(task.get("go_safe_after_cancel", True)),
                group_config=group_config,
            )
            msg = "任务组执行请求已发送"
            self.taskStatusChanged.emit(msg)
            return _dumps({"ok": True, "message": msg})
        except Exception as e:
            return _dumps({"ok": False, "message": str(e)})

    @Slot()
    def cancel_task(self) -> None:
        if self.pick_client:
            self.pick_client.cancel()
            self.taskStatusChanged.emit("已请求取消当前任务")

    @Slot()
    def go_home(self) -> None:
        if not self.pick_client:
            self.taskStatusChanged.emit("未连接 /simple_move_arm")
            return
        try:
            self.pick_client.go_home(done_cb=self._on_go_home_done)
            self.taskStatusChanged.emit("正在返回安全区")
        except Exception as e:
            self.taskStatusChanged.emit(f"返回安全区失败：{e}")

    def _on_pick_feedback(self, feedback) -> None:
        try:
            self.taskStatusChanged.emit(
                f"{feedback.stage_text} | 步骤 {feedback.current_step_index}/{feedback.total_steps}"
            )
        except Exception:
            pass

    def _on_pick_done(self, state, result) -> None:
        try:
            msg = getattr(result, "message", "")
            success = bool(getattr(result, "success", False))
            if success:
                self.taskStatusChanged.emit(f"任务完成：{msg}")
            elif bool(getattr(result, "canceled", False)):
                self.taskStatusChanged.emit(f"任务已取消：{msg}")
            else:
                self.taskStatusChanged.emit(f"任务失败：{msg}")
        except Exception:
            self.taskStatusChanged.emit(f"任务结束，状态={state}")

    def _on_go_home_done(self, state, result) -> None:
        try:
            if bool(getattr(result, "success", False)):
                self.taskStatusChanged.emit("返回安全区成功")
            else:
                self.taskStatusChanged.emit(
                    f"返回安全区失败：{getattr(result, 'message', '')}"
                )
        except Exception:
            self.taskStatusChanged.emit(f"返回安全区结束，状态={state}")
