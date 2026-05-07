# tomato_car_description

番茄采摘整车模型与 MoveIt 配置工作区；v0.2.0 起，系统定位从“单 PiPER 机械臂原型”升级为“整车 + PiPER 机械臂 + 三相机”的采摘平台

## 包结构

```text
tomato_car_description/
└── src/
    ├── tomato_car_description/   # 整车 URDF / xacro / meshes
    └── tomato_car_moveit/        # 基于整车模型生成的 MoveIt 配置
```

## 模型内容

- 底盘与四轮模型
- PiPER 机械臂模型
- 腕上 Gemini335L / 336L 相机安装模型
- 中景相机安装模型
- 远景相机安装模型
- `base_link`、车轮、机械臂和相机相关 TF 链

## 启动

显示整车模型：

```bash
source piper_tomato/devel/setup.bash
roslaunch tomato_car_description display.launch
```

启动整车 MoveIt：

```bash
source piper_tomato/devel/setup.bash
roslaunch tomato_car_moveit demo.launch
```

`piper_tomato/src/deployment/piper_bringup/launch/piper_start.launch` 会在系统启动时引用 `tomato_car_moveit`

