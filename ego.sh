cd EGO-Planner-v3
source devel/setup.bash
#!/bin/bash

# EGO-Planner 随机化初始位置启动脚本
# 使用方法:

echo "=========================================="
echo "  EGO-Planner 随机初始位置启动脚本"
echo "=========================================="

# 使用说明
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    echo "使用方法: $0 [X] [Y] [Z] [X_RANGE] [Y_RANGE] [Z_RANGE] [INIT_YAW]"
    echo "参数说明:"
    echo "  X: 中心X坐标 (默认: -4.5)"
    echo "  Y: 中心Y坐标 (默认: 0.1)"
    echo "  Z: 中心Z坐标 (默认: 1.7)"
    echo "  X_RANGE: X坐标范围半径 (默认: 0.1)"
    echo "  Y_RANGE: Y坐标范围半径 (默认: 0.2)"
    echo "  Z_RANGE: Z坐标范围半径 (默认: 0.15)"
    echo "  INIT_YAW: 初始yaw角度，弧度制 (默认: 3.14159265，即180度)"
    echo "示例: $0 -4.5 0.0 1.5 0.2 0.3 0.2 1.57"
    echo "这将在点(-4.5, 0.0, 1.5)周围生成随机位置，X范围±0.2，Y范围±0.3，Z范围±0.2，初始yaw角度为1.57弧度(90度)"
    exit 0
fi

# 从命令行参数读取中心点坐标和范围，如果没有提供则使用默认值
CENTER_X=${1:-(-4.5)}
CENTER_Y=${2:-0.1}
CENTER_Z=${3:-1.7}
RANGE_X=${4:-0.1}
RANGE_Y=${5:-0.2}
RANGE_Z=${6:-0.15}
INIT_YAW=${7:-3.14159265}  # 默认为π弧度(180度)

# 计算实际的最小最大值
X_MIN=$(python3 -c "print(f'{$CENTER_X - $RANGE_X:.2f}')")
X_MAX=$(python3 -c "print(f'{$CENTER_X + $RANGE_X:.2f}')")
Y_MIN=$(python3 -c "print(f'{$CENTER_Y - $RANGE_Y:.2f}')")
Y_MAX=$(python3 -c "print(f'{$CENTER_Y + $RANGE_Y:.2f}')")
Z_MIN=$(python3 -c "print(f'{$CENTER_Z - $RANGE_Z:.2f}')")
Z_MAX=$(python3 -c "print(f'{$CENTER_Z + $RANGE_Z:.2f}')")

# 生成随机坐标
INIT_X=$(python3 -c "import random; print(f'{random.uniform($X_MIN, $X_MAX):.2f}')")
INIT_Y=$(python3 -c "import random; print(f'{random.uniform($Y_MIN, $Y_MAX):.2f}')")
INIT_Z=$(python3 -c "import random; print(f'{random.uniform($Z_MIN, $Z_MAX):.2f}')")

echo "使用的参数:"
echo "  中心点: ($CENTER_X, $CENTER_Y, $CENTER_Z)"
echo "  X范围: [$X_MIN, $X_MAX] (±$RANGE_X)"
echo "  Y范围: [$Y_MIN, $Y_MAX] (±$RANGE_Y)"
echo "  Z范围: [$Z_MIN, $Z_MAX] (±$RANGE_Z)"
echo ""
echo "生成的随机初始位置:"
echo "  init_x: $INIT_X"
echo "  init_y: $INIT_Y" 
echo "  init_z: $INIT_Z"
echo "  init_yaw: $INIT_YAW (弧度)"
echo "=========================================="

# 等待2秒让用户看到随机位置
sleep 2
# roslaunch mission_fsm rviz.launch
sleep 2

# 启动 EGO-Planner，使用生成的随机位置
echo "正在启动 EGO-Planner..."
roslaunch mission_fsm multidrone_sim.launch \
    init_x:=$INIT_X \
    init_y:=$INIT_Y \
    init_z:=$INIT_Z \




