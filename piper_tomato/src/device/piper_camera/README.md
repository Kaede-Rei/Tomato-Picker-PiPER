# piper_camera

Orbbec 相机接入包，用于番茄采摘系统的腕上近景、中景外参和远景预测三相机配置；节点发布 RGB、Depth、Depth Registered / Depth to Color、CameraInfo 和 LRM 单点距离

## 启动

```bash
source piper_tomato/devel/setup.bash
roslaunch piper_camera piper_camera.launch
```

常用参数：

- `use_wrist`：启用腕上近景相机
- `use_mid`：启用中景外参相机
- `use_far`：启用远景预测相机
- `*_usb_port` / `*_index`：按 USB 端口或设备序号选择相机

## 默认输出

- `/piper/camera/wrist/color/image_raw`
- `/piper/camera/wrist/depth_registered/image_raw`
- `/piper/camera/wrist/lrm_distance`
- `/piper/camera/mid/color/image_raw`
- `/piper/camera/mid/depth_to_color`
- `/piper/camera/far/color/image_raw`
- `/piper/camera/far/depth_to_color`

本包基于 Orbbec SDK / pyorbbecsdk，并与工作区中的 OrbbecSDK_ROS1 生态配合使用

