#!/bin/bash
cd /home/bozhi/Desktop/DataCollect/Fast-Perching
source devel/setup.bash

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

roslaunch planning perching_unity.launch fixed_yaw:=$CURRENT_YAW
