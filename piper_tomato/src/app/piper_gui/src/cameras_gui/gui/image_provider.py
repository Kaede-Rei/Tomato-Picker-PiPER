from PySide6.QtGui import QImage
from PySide6.QtQuick import QQuickImageProvider
from threading import RLock

import os
import yaml
import numpy as np


def load_gui_config():
    config_path = os.path.join(
        os.path.dirname(__file__), "../../../config/cameras_gui.yaml"
    )
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
            gui_config = config.get("gui", {})

            width = int(gui_config.get("width", 1280))
            height = int(gui_config.get("height", 720))
            bg_color = gui_config.get("bg_color", "0x172033")
            if isinstance(bg_color, str):
                bg_color = bg_color.replace("#", "0x")
                bg_color = int(bg_color, 16)
            else:
                bg_color = int(bg_color)

            return width, height, bg_color

    except Exception as e:
        print(f"Failed to load gui config: {e}")

        return 1280, 720, 0x172033


class CamerasImageProvider(QQuickImageProvider):
    def __init__(self):
        super().__init__(QQuickImageProvider.ImageType.Image)
        self._lock = RLock()
        self._images: dict[str, QImage] = {}

        self._fallback_width, self._fallback_height, self._fallback_bg_color = (
            load_gui_config()
        )

    def set_bgr(self, camera_name: str, frame) -> None:
        with self._lock:
            if frame is None:
                self._images[camera_name] = QImage()
                return

            rgb = np.ascontiguousarray(frame[:, :, ::-1])
            h, w = rgb.shape[:2]
            image = QImage(rgb.data, w, h, rgb.strides[0], QImage.Format.Format_RGB888)
            self._images[camera_name] = image.copy()

    def requestImage(self, image_id, size, requested_size):  # override
        camera_name = str(image_id).split("?")[0].strip("/") or "wrist"

        with self._lock:
            image = self._images.get(camera_name, QImage())

        if image.isNull():
            fallback = QImage(
                self._fallback_width,
                self._fallback_height,
                QImage.Format.Format_RGB888,
            )
            fallback.fill(self._fallback_bg_color)
            return fallback

        return image
