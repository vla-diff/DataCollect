SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/ROS-Unity_bridge"
source devel/setup.bash
roslaunch ros_tcp_endpoint endpoint.launch  
