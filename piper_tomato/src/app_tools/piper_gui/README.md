# piper_gui

PiPER 番茄采摘多相机 GUI；当前主入口是 PySide6/QML 版本，支持腕上近景、中景外参、远景预测三路相机预览与切换，提供 ROI 框选、深度取点、TCP 平移补偿、PickTask 写入、任务组执行和回安全位操作

## 启动

```bash
source piper_tomato/devel/setup.bash
roslaunch piper_gui cameras_gui.launch
```

等价脚本：

```bash
rosrun piper_gui cameras_gui_qml.py
```

## 字体

GUI 默认优先使用仓库内附带的两个字体：

- `ttf/MapleMonoNormal-NF-CN-Regular.ttf`：中文 UI 与常规界面文本
- `ttf/JetBrainsMono-Medium.ttf`：日志、坐标和等宽数字显示

建议安装到用户字体目录：

```bash
mkdir -p ~/.local/share/fonts/tomato-picker-piper
cp piper_tomato/src/app_tools/piper_gui/ttf/*.ttf ~/.local/share/fonts/tomato-picker-piper/
fc-cache -fv
```

如果不安装，Qt 会按系统字体栈回退，功能不受影响，但界面字宽和中英文混排效果可能不同

## 多相机配置

默认配置文件：

```text
config/cameras_gui.yaml
```

默认相机：

| 键名 | 角色 | 默认 target frame | 说明 |
|---|---|---|---|
| `wrist` | 腕上近景 | `arm_link_tcp` | 近距离 ROI 选点，可优先使用 LRM |
| `mid` | 中景外参相机 | `base_link` | 中距离观测与目标定位 |
| `far` | 外景预测相机 | `base_link` | 远距离候选目标/场景观测 |

## TCP 补偿

任务页提供“TCP 平移补偿”开关和 `X/Y/Z` 输入，单位为米；ROI 提交时 GUI 会把当前补偿状态发送给后端；当目标输出坐标系等于 `frames.tcp_frame` 时，后端把补偿量直接加到目标点后再写入 `/pick_action`

该补偿用于标定残差兜底，不修改 TF，也不扩展 Action 消息字段

