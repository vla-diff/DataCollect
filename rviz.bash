SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/EGO-Planner-v3/"
source devel/setup.bash
roslaunch mission_fsm rviz.launch
