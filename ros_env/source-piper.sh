# 激活 orbbec、piper_ros 和 piper_tomato 的 ROS 环境
# Usage: 
#   source ./source-piper.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/../orbbec/devel/setup.bash"
source "$SCRIPT_DIR/../piper_ros/devel/setup.bash"
source "$SCRIPT_DIR/../piper_tomato/devel/setup.bash"