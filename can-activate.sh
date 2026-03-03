#!/bin/bash

# 脚本名称：
#   can_activate.sh
#
# 功能说明：
#   用于检测并初始化 USB-CAN 设备，自动完成以下操作：
#   1. 检查系统依赖（ethtool / can-utils）
#   2. 检测系统中 CAN 接口数量
#   3. 根据 USB 硬件地址（可选）确定目标 CAN 接口
#   4. 设置 CAN 波特率并启用接口
#   5. 将接口重命名为指定的 CAN 名称（默认 can0）

##### 脚本运行模式检测 #####
# 如果脚本被 source，则使用 return
# 如果直接执行，则使用 exit
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    SCRIPT_MODE="sourced"
else
    SCRIPT_MODE="executed"
fi

##### 安全退出函数 #####
error_exit() {
    echo -e "\e[31m[错误] $1\e[0m"
    echo "-------------------脚本已中断------------------"
    if [ "$SCRIPT_MODE" = "sourced" ]; then
        return 1
    else
        exit 1
    fi
}

##### 参数解析 #####
TARGET_CAN_NAME="${1:-can0}"
TARGET_BITRATE="${2:-1000000}"
USB_ADDRESS="${3}"

echo "-------------------开始-----------------------"

##### 依赖检查 #####
command -v ethtool >/dev/null 2>&1 || error_exit "未检测到 ethtool，请执行：sudo apt install ethtool"
command -v candump >/dev/null 2>&1 || error_exit "未检测到 can-utils，请执行：sudo apt install can-utils"

echo "依赖检查通过：ethtool / can-utils"

##### CAN 接口数量检测 #####
CURRENT_CAN_COUNT=$(ip -o link show type can | wc -l)

[ "$CURRENT_CAN_COUNT" -eq 0 ] && error_exit "系统中未检测到任何 CAN 接口"

if [ "$CURRENT_CAN_COUNT" -ne 1 ] && [ -z "$USB_ADDRESS" ]; then
    echo "检测到多个 CAN 接口："
    for iface in $(ip -br link show type can | awk '{print $1}'); do
        BUS_INFO=$(sudo ethtool -i "$iface" | awk '/bus-info/ {print $2}')
        echo "  接口 $iface -> USB 地址 $BUS_INFO"
    done
    error_exit "存在多个 CAN 接口，请通过第三个参数指定 USB 硬件地址"
fi

##### 接口匹配 #####
if [ -n "$USB_ADDRESS" ]; then
    echo "使用指定 USB 硬件地址：$USB_ADDRESS"
    INTERFACE_NAME=""
    for iface in $(ip -br link show type can | awk '{print $1}'); do
        BUS_INFO=$(sudo ethtool -i "$iface" | awk '/bus-info/ {print $2}')
        if [ "$BUS_INFO" = "$USB_ADDRESS" ]; then
            INTERFACE_NAME="$iface"
            break
        fi
    done
    [ -z "$INTERFACE_NAME" ] && error_exit "未找到 USB 地址为 $USB_ADDRESS 的 CAN 接口"
else
    INTERFACE_NAME=$(ip -br link show type can | awk '{print $1}')
fi

##### 已激活状态判断 #####
IS_UP=$(ip link show "$INTERFACE_NAME" | grep -q "UP" && echo "yes" || echo "no")
CURRENT_BITRATE=$(ip -details link show "$INTERFACE_NAME" | grep -oP 'bitrate \K\d+')
CURRENT_NAME="$INTERFACE_NAME"

if [ "$IS_UP" = "yes" ] \
   && [ "$CURRENT_BITRATE" = "$TARGET_BITRATE" ] \
   && [ "$CURRENT_NAME" = "$TARGET_CAN_NAME" ]; then
    echo "CAN 接口 $TARGET_CAN_NAME 已处于激活状态（bitrate=$TARGET_BITRATE），无需重复配置"
    echo "-------------------结束------------------------"
    return 0 2>/dev/null || exit 0
fi

##### 接口配置 #####
echo "配置 CAN 接口：$INTERFACE_NAME -> $TARGET_CAN_NAME（$TARGET_BITRATE）"

sudo ip link set "$INTERFACE_NAME" down || error_exit "关闭接口失败"
sudo ip link set "$INTERFACE_NAME" type can bitrate "$TARGET_BITRATE" || error_exit "设置波特率失败"

##### 接口重命名 #####
if [ "$INTERFACE_NAME" != "$TARGET_CAN_NAME" ]; then
    sudo ip link set "$INTERFACE_NAME" name "$TARGET_CAN_NAME" || error_exit "接口重命名失败"
fi

sudo ip link set "$TARGET_CAN_NAME" up || error_exit "启动接口失败"

echo "CAN 接口 $TARGET_CAN_NAME 已成功配置并激活"
echo "-------------------结束------------------------"
