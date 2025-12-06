# 检查命令行参数
if [ $# -lt 6 ]; then
    echo "用法: $0 <模式> <参数2> <参数3> <参数4> <参数5> <参数6> <参数7>"
    echo "模式1 (长方形区域随机): $0 0 <x> <y> <z> <x_range> <y_range> <z_range>"
    echo "模式2 (扇形区域随机): $0 1 <中心x> <中心y> <中心z> <小角theta1> <大角theta2> <半径>"
    echo "示例:"
    echo "  长方形模式: $0 0 6.25 -0.9 0.85 0.1 0.1 0.1"
    echo "  扇形模式: $0 1 12.8 -6.5 1.5 0.03 0.03 0.3"
    exit 1
fi

# 获取基本参数
MODE=${1:-1}
PARAM2=$2
PARAM3=$3
PARAM4=$4
PARAM5=$5
PARAM6=$6
PARAM7=$7

# 根据模式解析参数
if [ "$MODE" -eq 0 ]; then
    # 长方形区域随机模式
    TARGET_X=$PARAM2
    TARGET_Y=$PARAM3
    TARGET_Z=$PARAM4
    X_RANGE=$PARAM5
    Y_RANGE=$PARAM6
    Z_RANGE=$PARAM7

    echo "模式: 长方形区域随机"
    echo "目标位置坐标: X=$TARGET_X, Y=$TARGET_Y, Z=$TARGET_Z"
    echo "坐标范围: X_RANGE=$X_RANGE, Y_RANGE=$Y_RANGE, Z_RANGE=$Z_RANGE"

elif [ "$MODE" -eq 1 ]; then
    # 扇形区域随机模式
    CENTER_X=$PARAM2
    CENTER_Y=$PARAM3
    CENTER_Z=$PARAM4
    THETA1=$PARAM5    # 小角
    THETA2=$PARAM6    # 大角
    RADIUS=$PARAM7    # 半径

    echo "模式: 扇形区域随机"
    echo "圆心坐标: X=$CENTER_X, Y=$CENTER_Y, Z=$CENTER_Z"
    echo "角度范围: $THETA1 ~ $THETA2 弧度"
    echo "半径: $RADIUS 米"

else
    echo "错误: 无效的模式 $MODE，必须是0或1"
    exit 1
fi

# 从/unity_depth_odom0话题获取当前位置和姿态
echo "正在从/unity_depth_odom0话题获取当前位置和姿态信息..."

# 等待并获取一条odom消息，超时时间为10秒
ODOM_DATA=$(timeout 10 rostopic echo -n 1 /unity_depth_odom0)

# 检查是否成功获取数据
if [ -z "$ODOM_DATA" ]; then
    echo "错误: 无法从/unity_depth_odom0话题获取数据! 请检查话题是否存在或数据是否正在发布。"
    exit 1
fi

echo "成功获取odom数据"

# 提取位置信息并清理数据格式
X_POS_RAW=$(echo "$ODOM_DATA" | grep -A 1 "position:" | grep "x:" | sed 's/.*x: *//' | tr -d ' ')
Y_POS_RAW=$(echo "$ODOM_DATA" | grep -A 2 "position:" | grep "y:" | sed 's/.*y: *//' | tr -d ' ')
Z_POS_RAW=$(echo "$ODOM_DATA" | grep -A 3 "position:" | grep "z:" | sed 's/.*z: *//' | tr -d ' ')

# 清理和格式化位置数据，确保为有效浮点数
X_POS=$(python3 -c "
try:
    val = '$X_POS_RAW'.strip()
    print(f'{float(val):.6f}' if val else '0.0')
except:
    print('0.0')
")

Y_POS=$(python3 -c "
try:
    val = '$Y_POS_RAW'.strip()
    print(f'{float(val):.6f}' if val else '0.0')
except:
    print('0.0')
")

Z_POS=$(python3 -c "
try:
    val = '$Z_POS_RAW'.strip()
    print(f'{float(val):.6f}' if val else '0.0')
except:
    print('0.0')
")

echo "提取到的当前位置坐标: X=$X_POS, Y=$Y_POS, Z=$Z_POS"

# 提取四元数姿态信息并清理数据格式
QUAT_X_RAW=$(echo "$ODOM_DATA" | grep -A 1 "orientation:" | grep "x:" | sed 's/.*x: *//' | tr -d ' ')
QUAT_Y_RAW=$(echo "$ODOM_DATA" | grep -A 2 "orientation:" | grep "y:" | sed 's/.*y: *//' | tr -d ' ')
QUAT_Z_RAW=$(echo "$ODOM_DATA" | grep -A 3 "orientation:" | grep "z:" | sed 's/.*z: *//' | tr -d ' ')
QUAT_W_RAW=$(echo "$ODOM_DATA" | grep -A 4 "orientation:" | grep "w:" | sed 's/.*w: *//' | tr -d ' ')

# 清理和格式化四元数数据，确保为有效浮点数
QUAT_X=$(python3 -c "
try:
    val = '$QUAT_X_RAW'.strip()
    print(f'{float(val):.6f}' if val else '0.0')
except:
    print('0.0')
")

QUAT_Y=$(python3 -c "
try:
    val = '$QUAT_Y_RAW'.strip()
    print(f'{float(val):.6f}' if val else '0.0')
except:
    print('0.0')
")

QUAT_Z=$(python3 -c "
try:
    val = '$QUAT_Z_RAW'.strip()
    print(f'{float(val):.6f}' if val else '0.0')
except:
    print('0.0')
")

QUAT_W=$(python3 -c "
try:
    val = '$QUAT_W_RAW'.strip()
    print(f'{float(val):.6f}' if val else '1.0')
except:
    print('1.0')
")

echo "提取到的四元数: X=$QUAT_X, Y=$QUAT_Y, Z=$QUAT_Z, W=$QUAT_W"

# 根据模式计算带有随机偏移的最终坐标
if [ "$MODE" -eq 0 ]; then
    # 长方形区域随机模式
    FINAL_X=$(python3 -c "
import random
target = float('$TARGET_X')
range_val = float('$X_RANGE')
offset = random.uniform(-range_val, range_val)
final_x = target + offset
print(f'{final_x:.6f}')
")

    FINAL_Y=$(python3 -c "
import random
target = float('$TARGET_Y')
range_val = float('$Y_RANGE')
offset = random.uniform(-range_val, range_val)
final_y = target + offset
print(f'{final_y:.6f}')
")

    FINAL_Z=$(python3 -c "
import random
target = float('$TARGET_Z')
range_val = float('$Z_RANGE')
offset = random.uniform(-range_val, range_val)
final_z = target + offset
print(f'{final_z:.6f}')
")

    echo "长方形模式 - 计算得到的最终坐标: X=$FINAL_X, Y=$FINAL_Y, Z=$FINAL_Z"

elif [ "$MODE" -eq 1 ]; then
    # 扇形区域随机模式
    # 在扇形内生成随机点
    SECTOR_RESULT=$(python3 -c "
import random
import math

center_x = float('$CENTER_X')
center_y = float('$CENTER_Y')
center_z = float('$CENTER_Z')
theta1 = float('$THETA1')
theta2 = float('$THETA2')
radius = float('$RADIUS')

# 随机角度 (在theta1和theta2之间)
random_theta = random.uniform(theta1, theta2)
# 随机半径 (使用平方根分布保证点在扇形内均匀分布)
random_r = math.sqrt(random.random()) * radius

# 计算最终坐标 (在XY平面内的扇形)
final_x = center_x + random_r * math.cos(random_theta)
final_y = center_y + random_r * math.sin(random_theta)
final_z = center_z  # Z坐标保持不变

print(f'{final_x:.6f} {final_y:.6f} {final_z:.6f}')
")

    # 解析Python输出的结果
    FINAL_X=$(echo "$SECTOR_RESULT" | cut -d' ' -f1)
    FINAL_Y=$(echo "$SECTOR_RESULT" | cut -d' ' -f2)
    FINAL_Z=$(echo "$SECTOR_RESULT" | cut -d' ' -f3)

    echo "扇形模式 - 计算得到的最终坐标: X=$FINAL_X, Y=$FINAL_Y, Z=$FINAL_Z"
fi

rostopic pub -1 /triger geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: $FINAL_X
    y: $FINAL_Y
    z: $FINAL_Z
  orientation:
    x: $QUAT_X
    y: $QUAT_Y
    z: $QUAT_Z
    w: $QUAT_W"
