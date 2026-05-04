from distutils import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["cameras_gui", "cameras_gui.core", "cameras_gui.ros", "cameras_gui.ros"],
    package_dir={"": "src"},
    package_data={
        "cameras_gui": ["gui/qml/*.qml"],
    },
)

setup(**setup_args)
