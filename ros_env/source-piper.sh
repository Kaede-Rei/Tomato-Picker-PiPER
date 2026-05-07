# 激活 orbbec、piper_ros 和 piper_tomato 的 ROS 环境
# Usage: 
#   source ./source-piper.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for setup in \
    "$SCRIPT_DIR/../external/orbbec/devel/setup.bash" \
    "$SCRIPT_DIR/../external/piper_ros/devel/setup.bash" \
    "$SCRIPT_DIR/../piper_tomato/devel/setup.bash"; do
    if [ -f "$setup" ]; then
        source "$setup" --extend
    else
        echo "[WARN] ROS setup not found: $setup" >&2
    fi
done
