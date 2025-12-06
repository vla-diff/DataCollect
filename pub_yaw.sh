#!/bin/bash

# 使用说明：
# ./pub_yaw.sh [target_yaw]
# target_yaw: 可选参数，目标的绝对yaw角度（弧度）
# 例如：./pub_yaw.sh 1.57  # 设置目标yaw为1.57弧度（90度）
# 例如：./pub_yaw.sh -1.57 # 设置目标yaw为-1.57弧度（-90度）

# Source ROS workspace
cd /home/bozhi/Desktop/DataCollect/EGO-Planner-v3
source /home/bozhi/Desktop/DataCollect/EGO-Planner-v3/devel/setup.bash

# 函数：将四元数转换为欧拉角
quat_to_euler() {
    local qx=$1 qy=$2 qz=$3 qw=$4
    echo "调试 - 输入四元数: qx=$qx, qy=$qy, qz=$qz, qw=$qw" >&2
    
    # 使用Python进行完整的四元数到欧拉角转换
    python3 -c "
import math

qx, qy, qz, qw = $qx, $qy, $qz, $qw

# Roll (x-axis rotation)
sinr_cosp = 2 * (qw * qx + qy * qz)
cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
roll = math.atan2(sinr_cosp, cosr_cosp)

# Pitch (y-axis rotation)
sinp = 2 * (qw * qy - qz * qx)
if abs(sinp) >= 1:
    pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
else:
    pitch = math.asin(sinp)

# Yaw (z-axis rotation)
siny_cosp = 2 * (qw * qz + qx * qy)
cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
yaw = math.atan2(siny_cosp, cosy_cosp)

print(f'调试 - Roll: {roll:.6f}, Pitch: {pitch:.6f}, Yaw: {yaw:.6f}', file=__import__('sys').stderr)
print(f'{roll} {pitch} {yaw}')
"
}

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

# 计算欧拉角
EULER_ANGLES=$(quat_to_euler $QUAT_X $QUAT_Y $QUAT_Z $QUAT_W)
ROLL_ANGLE=$(echo $EULER_ANGLES | cut -d' ' -f1)
PITCH_ANGLE=$(echo $EULER_ANGLES | cut -d' ' -f2)
YAW_ANGLE=$(echo $EULER_ANGLES | cut -d' ' -f3)

echo "欧拉角 - Roll: $ROLL_ANGLE, Pitch: $PITCH_ANGLE, Yaw: $YAW_ANGLE"

# 使用实际的yaw角度（不是roll）
CURRENT_YAW=$YAW_ANGLE
echo "当前yaw角度: $CURRENT_YAW 弧度"

# 设置目标yaw角度（绝对值）
if [ $# -eq 1 ]; then
    TARGET_YAW_INPUT=$1
    echo "使用命令行参数设置目标yaw角度: $TARGET_YAW_INPUT 弧度"
    
    # 将输入的目标yaw角度限制在[-π, π]范围内
    TARGET_YAW=$(python3 -c "
import math
target = float('$TARGET_YAW_INPUT')
# 将角度限制在[-π, π]范围内
while target > math.pi:
    target -= 2 * math.pi
while target < -math.pi:
    target += 2 * math.pi
print(f'{target:.6f}')
")
else
    # 默认目标yaw为当前yaw（不旋转）
    TARGET_YAW=$CURRENT_YAW
    echo "未指定目标yaw，使用当前yaw角度: $TARGET_YAW 弧度"
fi

echo "目标yaw角度（限制在-π到π）: $TARGET_YAW 弧度"

# 计算需要旋转的角度差（目标yaw - 当前yaw），并限制在[-π, π]范围内
ANGLE_DIFF=$(python3 -c "
import math
current = float('$CURRENT_YAW')
target = float('$TARGET_YAW')
# 计算角度差（目标 - 当前）
diff = target - current
# 将角度差限制在[-π, π]范围内，选择最短路径
while diff > math.pi:
    diff -= 2 * math.pi
while diff < -math.pi:
    diff += 2 * math.pi
print(f'{diff:.6f}')
")

echo "需要旋转的角度: $ANGLE_DIFF 弧度"

# 设置插值参数
MAX_STEP_ANGLE=0.3  # 每步最大角度变化（约17度）
PUBLISH_FREQUENCY=10  # 发布频率（Hz）
SLEEP_TIME=$(python3 -c "print(f'{1.0 / $PUBLISH_FREQUENCY:.3f}')")  # 计算休眠时间

# 计算需要的步数
STEPS=$(python3 -c "
import math
angle_diff = abs(float('$ANGLE_DIFF'))
max_step = float('$MAX_STEP_ANGLE')
steps = max(1, int(math.ceil(angle_diff / max_step)))
print(steps)
")

echo "将分 $STEPS 步执行，每步间隔 $SLEEP_TIME 秒"

# 逐步插值发布yaw角度
for ((i=1; i<=STEPS; i++)); do
    # 计算当前步骤的yaw角度
    PROGRESS=$(python3 -c "print($i / $STEPS)")
    CURRENT_STEP_YAW=$(python3 -c "
import math
current = float('$CURRENT_YAW')
angle_diff = float('$ANGLE_DIFF')
progress = float('$PROGRESS')
step_yaw = current + angle_diff * progress
# 限制在[-π, π]范围内
while step_yaw > math.pi:
    step_yaw -= 2 * math.pi
while step_yaw < -math.pi:
    step_yaw += 2 * math.pi
print(f'{step_yaw:.6f}')
")
    
    echo "步骤 $i/$STEPS: 发布yaw角度 $CURRENT_STEP_YAW 弧度"
    
    # 发布位置命令，确保所有数值都是浮点数格式
    rostopic pub --once /drone_0_planning/pos_cmd quadrotor_msgs/PositionCommand "{
    header: {seq: $i, stamp: now, frame_id: 'world'},
    position: {x: ${X_POS}, y: ${Y_POS}, z: ${Z_POS}},
    velocity: {x: 0.0, y: 0.0, z: 0.0},
    acceleration: {x: 0.0, y: 0.0, z: 0.0},
    jerk: {x: 0.0, y: 0.0, z: 0.0},
    yaw: ${CURRENT_STEP_YAW},
    yaw_dot: 0.0,
    trajectory_id: 5,
    trajectory_flag: 1
    }" &

    # 发布位置命令，确保所有数值都是浮点数格式
    rostopic pub --once /goal_with_id_from_station quadrotor_msgs/GoalSet "
    to_drone_ids: [0]
    goal:
    - {x: ${X_POS}, y: ${Y_POS}, z: ${Z_POS}}
    yaw: [${CURRENT_STEP_YAW}]
    look_forward: false
    goal_to_follower: false" &
    
    # 如果不是最后一步，等待一段时间
    if [ $i -lt $STEPS ]; then
        sleep $SLEEP_TIME
    fi
done

# 等待所有后台任务完成
wait

echo "插值发布完成！最终yaw角度: $CURRENT_STEP_YAW 弧度"
