# 激活 piper_ros 和 piper_controller 的 ROS 环境
# Usage: 
#   source ./source-piper.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/../piper_ros/devel/setup.bash"
source "$SCRIPT_DIR/../piper_controller/devel/setup.bash"