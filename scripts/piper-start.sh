#!/usr/bin/env bash

# 启动 Piper 系统脚本，并在收到中断时执行回零（可选失能）
#
# 使用方法：
#   ./piper-start.sh [--fake|--real] [--disable] [--delay 秒数]
#   ./piper-start.sh [--fake|-f] [--real|-r] [-d] [-t 秒数]
#
# 示例：
#   ./piper-start.sh
#   ./piper-start.sh --fake
#   ./piper-start.sh --real
#   ./piper-start.sh --disable 或 ./piper-start.sh -d
#   ./piper-start.sh --delay 5 或 ./piper-start.sh -t 5
#   ./piper-start.sh --disable --delay 8
#
# 参数：
#   --fake, -f      使用 MoveIt fake_controller，不启动真机控制节点，不配置 CAN
#   --real, -r      使用真机 controller（默认）
#   --disable, -d   中断时调用 /enable_srv 使系统失能，默认不失能
#   --delay, -t     回零后等待时间（秒），默认 2 秒

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ORBBEC_USB_RESOLVER="$SCRIPT_DIR/../piper_tomato/src/adapters/piper_camera_adapter/scripts/piper_orbbec_usb.py"

# source ROS 环境。第一个工作区不使用 --extend，避免继承调用 shell 里的脏 overlay。
BASE_SETUP="$SCRIPT_DIR/../external/orbbec/devel/setup.bash"
if [ -f "$BASE_SETUP" ]; then
    source "$BASE_SETUP"
else
    echo "未找到 ROS 工作空间 setup: $BASE_SETUP，请先编译 ROS 包"
    exit 1
fi

for setup in \
    "$SCRIPT_DIR/../external/piper_ros/devel/setup.bash" \
    "$SCRIPT_DIR/../piper_tomato/devel/setup.bash"; do
    if [ -f "$setup" ]; then
        source "$setup" --extend
    else
        echo "未找到 ROS 工作空间 setup: $setup，请先编译 ROS 包"
        exit 1
    fi
done

# 读取 config.json 配置默认参数
CONFIG_FILE="$SCRIPT_DIR/config.json"
LAUNCH_ARGS=()

if [ -f "$CONFIG_FILE" ]; then
    # 获取系统参数
    DISABLE_ON_EXIT=$(python3 -c "import json, sys; print(str(json.load(open('$CONFIG_FILE')).get('system', {}).get('disable_on_exit', False)).lower())" 2>/dev/null || echo "false")
    DELAY_SEC=$(python3 -c "import json, sys; print(json.load(open('$CONFIG_FILE')).get('system', {}).get('delay_sec', 2))" 2>/dev/null || echo "2")
    USE_FAKE_CONTROLLER=$(python3 -c "import json, sys; print(str(json.load(open('$CONFIG_FILE')).get('launch_args', {}).get('use_fake_controller', False)).lower())" 2>/dev/null || echo "false")
    
    # 提取 launch 文件参数，构造成 kwarg:=value 的格式
    # 过滤掉空字符串参数（例如空 USB 口配置），以防给 rosparam 传递异常空串
    EXTRA_ARGS=$(python3 -c "
import json
try:
    d = json.load(open('$CONFIG_FILE')).get('launch_args', {})
    res = []
    for k, v in d.items():
        if isinstance(v, bool):
            val = 'true' if v else 'false'
        else:
            val = str(v)
        if val != '':
            res.append(f'{k}:={val}')
    print(' '.join(res))
except Exception as e:
    pass
")
    if [ -n "$EXTRA_ARGS" ]; then
        read -ra LAUNCH_ARGS <<< "$EXTRA_ARGS"
    fi
else
    # 万一找不到配置文件，回退到默认
    DISABLE_ON_EXIT=false
    DELAY_SEC=2
    USE_FAKE_CONTROLLER=false
fi

SUCCESS=false
ROSLAUNCH_PID=""
CLEANING_UP=false

# 解析 CLI 参数（支持重写配置中的同名标志和引入新 launch 参数）
while [[ $# -gt 0 ]]; do
    case "$1" in
        --fake|-f)
            USE_FAKE_CONTROLLER=true
            LAUNCH_ARGS+=("use_fake_controller:=true")
            shift
            ;;
        --real|-r)
            USE_FAKE_CONTROLLER=false
            LAUNCH_ARGS+=("use_fake_controller:=false")
            shift
            ;;
        --disable|-d)
            DISABLE_ON_EXIT=true
            shift
            ;;
        --delay|-t)
            if [[ -z "${2:-}" ]]; then
                echo "错误: --delay 需要一个秒数参数"
                exit 1
            fi
            DELAY_SEC="$2"
            shift 2
            ;;
        --help|-h)
            grep -E '^#' "$0" | sed 's/^# //'
            echo ""
            echo "进阶：你可以传递任何额外的 roslaunch 参数来覆盖 config.json 配置，例如:"
            echo "  ./piper-start.sh use_mid_camera:=true wrist_usb_port:=/dev/video0"
            exit 0
            ;;
        *)
            # 捕获类似 use_camera:=true 这样的额外参数，传递给 roslaunch
            if [[ "$1" == *":="* ]]; then
               LAUNCH_ARGS+=("$1")
               # 如果用户显式传了 use_fake_controller，我们也要更新 bash 变量以维持逻辑一致
               if [[ "$1" == "use_fake_controller:=true" ]]; then
                   USE_FAKE_CONTROLLER=true
               elif [[ "$1" == "use_fake_controller:=false" ]]; then
                   USE_FAKE_CONTROLLER=false
               fi
            else
               echo "未知参数或格式错误（需满足 key:=value 格式）: $1"
               exit 1
            fi
            shift
            ;;
    esac
done

resolve_orbbec_usb_launch_args() {
    if [ ! -x "$ORBBEC_USB_RESOLVER" ]; then
        echo "[ERROR] 未找到 Orbbec USB 端口解析器: $ORBBEC_USB_RESOLVER"
        exit 1
    fi

    local resolved_args=()
    local item key value resolved
    for item in "${LAUNCH_ARGS[@]}"; do
        if [[ "$item" =~ ^(wrist_usb_port|mid_usb_port|far_usb_port):=(.+)$ ]]; then
            key="${BASH_REMATCH[1]}"
            value="${BASH_REMATCH[2]}"
            if [[ -n "$value" ]]; then
                if ! resolved="$(python3 "$ORBBEC_USB_RESOLVER" --strict "$value")"; then
                    echo "[ERROR] $key 无法解析为 Orbbec SDK USB UID: $value"
                    exit 1
                fi
                if [[ "$resolved" != "$value" ]]; then
                    echo "[USB] $key: $value -> $resolved"
                fi
                resolved_args+=("$key:=$resolved")
                continue
            fi
        fi
        resolved_args+=("$item")
    done
    LAUNCH_ARGS=("${resolved_args[@]}")
}

resolve_orbbec_usb_launch_args

cleanup() {
    # 防止重复进入 cleanup
    if [[ "$CLEANING_UP" == true ]]; then
        return
    fi
    CLEANING_UP=true

    if [[ "$SUCCESS" == false ]]; then
        echo "PiPER 未成功启动，直接退出"
        exit 1
    fi

    echo "检测到中断，尝试令机械臂回到零点 ..."
    python3 - <<'PY' || echo "跳过回零动作（服务未就绪）"
import sys
import actionlib
import rospy
from piper_contract.msg import SimpleMoveArmAction, SimpleMoveArmGoal

def main():
    rospy.init_node("piper_exit_zero_checker", anonymous=True, disable_signals=True)
    client = actionlib.SimpleActionClient("/piper/simple_move_arm", SimpleMoveArmAction)

    if not client.wait_for_server(rospy.Duration(1.5)):
        return 1

    print("检测到服务，正在令机械臂回到零点...")
    goal = SimpleMoveArmGoal()
    goal.command_type = SimpleMoveArmGoal.MOVE_TO_ZERO
    client.send_goal(goal)

    if not client.wait_for_result(rospy.Duration(10.0)):
        client.cancel_goal()
        print("[WARN] 回零超时")
    return 0

if __name__ == "__main__":
    sys.exit(main())
PY

    sleep "$DELAY_SEC"

    if [[ "$DISABLE_ON_EXIT" == true ]]; then
        if rosservice list 2>/dev/null | grep -q "^/enable_srv$"; then
            echo "[ACTION] 正在使系统失能..."
            rosservice call /enable_srv "enable_request: false" >/dev/null 2>&1 || true
        fi
    fi

    if [[ -n "$ROSLAUNCH_PID" ]]; then
        echo "[ACTION] 正在关闭 roslaunch (PID: $ROSLAUNCH_PID)..."
        kill "$ROSLAUNCH_PID" 2>/dev/null || true
        sleep 1
        kill -9 "$ROSLAUNCH_PID" 2>/dev/null || true
    fi

    echo "退出完成"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "================ 启动 Piper 系统 ================"
if [[ "$USE_FAKE_CONTROLLER" == true ]]; then
    echo "[MODE] fake_controller"
else
    echo "[MODE] real controller"
fi

# 配置 CAN 接口
if [[ "$USE_FAKE_CONTROLLER" == true ]]; then
    echo "[1/2] fake 模式跳过 CAN 配置"
else
    echo "[1/2] 配置 CAN 接口"
    if [ -f "$SCRIPT_DIR/can-activate.sh" ]; then
        sudo "$SCRIPT_DIR/can-activate.sh" || echo "[WARN] CAN 配置脚本执行异常，尝试继续启动 ROS..."
    else
        echo "[WARN] 未找到 can-activate.sh，跳过配置步骤"
    fi
fi

# 启动 ROS Launch
echo "[2/2] 启动 ROS Launch (附带参数: ${LAUNCH_ARGS[*]})"
roslaunch piper_bringup piper_start.launch "${LAUNCH_ARGS[@]}" &
ROSLAUNCH_PID=$!

# 只要 roslaunch 成功拉起，cleanup 就可以接管退出流程
SUCCESS=true

# 等待子进程退出
wait "$ROSLAUNCH_PID" || true
