[RoboStack Documentation](https://robostack.github.io/)

```bash
# create a ros env and install the dependencies
micromamba create -n ros_env -c conda-forge -c robostack-noetic \
    ros-noetic-desktop-full \
    ros-dev-tools \
    ros-noetic-moveit \
    ros-noetic-pcl-ros \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-trac-ik-kinematics-plugin \
    ros-noetic-rosserial \
    ros-noetic-rosserial-python \
    ros-noetic-image-geometry \
    elfutils binutils sysroot_linux-64 \
    compilers cxx-compiler c-compiler

# if you want to install others
micromamba activate ros_env
micromamba install -c conda-forge -c robostack-noetic \
    ros-noetic-<package-name>

# install the OrbbecSDK and add the udev rules
cd ./orbbec/pyorbbecsdk && sudo chmod +x ./install_udev_rules.sh && sudo ./install_udev_rules.sh && sudo udevadm control --reload && sudo udevadm trigger && pip install pyorbbecsdk2 && cd ../../
# install Can and PiPER-SDK
pip install python-can piper_sdk
# optional trimesh can be used to simplify robotic arm meshes
pip install fast-simplification trimesh
# build the workspace
. ./ros_env/use-mamba-gcc.sh && cd tomato_car_description && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source devel/setup.bash && cd ../orbbec && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && source devel/setup.bash && cd ../piper_ros && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && source devel/setup.bash && cd ../piper_tomato && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 && source devel/setup.bash && cd ..
# or if you need clangd
. ./ros_env/use-mamba-gcc.sh && cd tomato_car_description && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source devel/setup.bash && cd ../orbbec && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source devel/setup.bash && cd ../piper_ros && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source devel/setup.bash && cd ../piper_tomato && catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source devel/setup.bash && cd .. && ln -sf ../piper_tomato/build/compile_commands.json ./build/compile_commands.json
# run the demo
cd .. && ./piper-start.sh
```