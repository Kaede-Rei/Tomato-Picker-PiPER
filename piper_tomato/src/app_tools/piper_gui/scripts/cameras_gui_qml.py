#!/usr/bin/env python3
import os
import site
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"

if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def _candidate_site_dirs() -> list[Path]:
    dirs: list[Path] = []

    for value in site.getsitepackages() + [site.getusersitepackages()]:
        if value:
            dirs.append(Path(value))

    for value in sys.path:
        if value:
            dirs.append(Path(value))

    return dirs


def _bootstrap_pyside6_qt() -> None:
    if os.environ.get("CAMERAS_GUI_QT_BOOTSTRAPPED") == "1":
        return

    for base in _candidate_site_dirs():
        qt_root = base / "PySide6" / "Qt"
        qt_lib = qt_root / "lib"
        qt_qml = qt_root / "qml"
        qt_plugins = qt_root / "plugins"

        if not (qt_lib / "libQt6Core.so.6").exists():
            continue

        env = os.environ.copy()
        env["CAMERAS_GUI_QT_BOOTSTRAPPED"] = "1"

        old_ld = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = str(qt_lib) + (os.pathsep + old_ld if old_ld else "")

        if qt_qml.exists():
            old_qml = env.get("QML2_IMPORT_PATH", "")
            env["QML2_IMPORT_PATH"] = str(qt_qml) + (
                os.pathsep + old_qml if old_qml else ""
            )

        if qt_plugins.exists():
            old_plugins = env.get("QT_PLUGIN_PATH", "")
            env["QT_PLUGIN_PATH"] = str(qt_plugins) + (
                os.pathsep + old_plugins if old_plugins else ""
            )

        env["QT_API"] = "pyside6"

        os.execvpe(sys.executable, [sys.executable, *sys.argv], env)

    return


_bootstrap_pyside6_qt()


import rospy
import rospkg
from PySide6.QtCore import QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine

from cameras_gui.gui.backend import CamerasGuiBackend
from cameras_gui.gui.image_provider import CamerasImageProvider


def main() -> None:
    rospack = rospkg.RosPack()
    pkg_path = Path(rospack.get_path("piper_gui"))

    default_config = pkg_path / "config" / "cameras_gui.yaml"

    config_path = Path(
        rospy.get_param("~config", str(default_config))
        if rospy.core.is_initialized()
        else str(default_config)
    )

    app = QGuiApplication(sys.argv)

    engine = QQmlApplicationEngine()

    image_provider = CamerasImageProvider()
    backend = CamerasGuiBackend(image_provider, config_path)

    engine.addImageProvider("cameras", image_provider)
    engine.rootContext().setContextProperty("backend", backend)

    qml_path = ROOT / "src" / "cameras_gui" / "gui" / "qml" / "Main.qml"

    if not qml_path.exists():
        qml_path = pkg_path / "gui" / "qml" / "Main.qml"

    engine.load(QUrl.fromLocalFile(str(qml_path)))

    if not engine.rootObjects():
        raise SystemExit("Failed to load QML.")

    raise SystemExit(app.exec())


if __name__ == "__main__":
    main()
