SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/lidar_filter/"
source devel/setup.bash
rosrun livox_radius_filter radius_filter_node
