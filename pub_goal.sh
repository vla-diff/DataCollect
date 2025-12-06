#!/bin/bash

# 随机目标点发送脚本（增强版）
# 用法: ./send_random_goal.sh <模式> <参数2> <参数3> <参数4> [参数5] [参数6] [参数7] [roll] [pitch] [yaw]
# 模式1 (长方形): <0> <x> <y> <z> [x范围] [y范围] [z范围] [roll] [pitch] [yaw]
# 模式2 (扇形): <1> <中心x> <中心y> <中心z> <小角theta1> <大角theta2> <半径> [roll] [pitch] [yaw]

# 检查参数
if [ $# -lt 4 ]; then
    echo "用法: $0 <模式> <参数2> <参数3> <参数4> [参数5] [参数6] [参数7] [roll] [pitch] [yaw]"
    echo "模式1 (长方形区域随机): $0 0 <x> <y> <z> [x范围] [y范围] [z范围] [roll] [pitch] [yaw]"
    echo "模式2 (扇形区域随机): $0 1 <中心x> <中心y> <中心z> <小角theta1> <大角theta2> <半径> [roll] [pitch] [yaw]"
    echo "例如:"
    echo "  长方形模式: $0 0 5.0 3.0 2.0 1.0 1.0 0.5 0.0 0.0 0.0"
    echo "  扇形模式: $0 1 12.8 -6.5 1.5 0.03 0.03 0.3 0.0 0.0 0.0"
    exit 1
fi

# 获取基本参数
MODE=${1:-1}
PARAM2=${2:-0}
PARAM3=${3:-0}
PARAM4=${4:-0}
PARAM5=${5:-0}
PARAM6=${6:-0}
PARAM7=${7:-0}
ROLL=${10:-0.0}
PITCH=${9:-0.0}
YAW=${8:-0.0}


if [ "$MODE" -eq 1 ]; then
    # 长方形区域随机模式
    BASE_X=$PARAM2
    BASE_Y=$PARAM3
    BASE_Z=$PARAM4
    RANDOM_RANGE_X=${PARAM5:-1.0}  # X轴默认随机范围1米
    RANDOM_RANGE_Y=${PARAM6:-1.0}  # Y轴默认随机范围1米
    RANDOM_RANGE_Z=${PARAM7:-0.5}  # Z轴默认随机范围0.5米

    echo "模式: 长方形区域随机"
    echo "基础目标点: ($BASE_X, $BASE_Y, $BASE_Z)"
    echo "X轴随机范围: ±$RANDOM_RANGE_X 米"
    echo "Y轴随机范围: ±$RANDOM_RANGE_Y 米"
    echo "Z轴随机范围: ±$RANDOM_RANGE_Z 米"
    echo "姿态角度: roll=$ROLL, pitch=$PITCH, yaw=$YAW"
    echo "=========================================="

    # 生成随机偏移量
    OFFSET_X=$(python3 -c "import random; print(f'{random.uniform(-$RANDOM_RANGE_X, $RANDOM_RANGE_X):.3f}')")
    OFFSET_Y=$(python3 -c "import random; print(f'{random.uniform(-$RANDOM_RANGE_Y, $RANDOM_RANGE_Y):.3f}')")
    OFFSET_Z=$(python3 -c "import random; print(f'{random.uniform(-$RANDOM_RANGE_Z, $RANDOM_RANGE_Z):.3f}')")

    # 计算最终坐标
    FINAL_X=$(python3 -c "print(f'{$BASE_X + $OFFSET_X:.3f}')")
    FINAL_Y=$(python3 -c "print(f'{$BASE_Y + $OFFSET_Y:.3f}')")
    FINAL_Z=$(python3 -c "print(f'{$BASE_Z + $OFFSET_Z:.3f}')")

    echo "随机偏移量: X=$OFFSET_X, Y=$OFFSET_Y, Z=$OFFSET_Z"

elif [ "$MODE" -eq 2 ]; then
    # 扇形区域随机模式
    CENTER_X=$PARAM2
    CENTER_Y=$PARAM3
    CENTER_Z=$PARAM4
    THETA1=$PARAM5    # 小角
    THETA2=$PARAM6    # 大角
    RADIUS=$PARAM7    # 半径

    echo "模式: 扇形区域随机"
    echo "圆心坐标: ($CENTER_X, $CENTER_Y, $CENTER_Z)"
    echo "角度范围: $THETA1 ~ $THETA2 弧度"
    echo "半径: $RADIUS 米"
    echo "姿态角度: roll=$ROLL, pitch=$PITCH, yaw=$YAW"
    echo "=========================================="

    # 在扇形内生成随机点
    # 随机角度 (在theta1和theta2之间)
    RANDOM_THETA=$(python3 -c "import random; print(f'{random.uniform($THETA1, $THETA2):.6f}')")
    # 随机半径 (使用平方根分布保证点在扇形内均匀分布)
    RANDOM_R=$(python3 -c "import random, math; print(f'{math.sqrt(random.random()) * $RADIUS:.3f}')")
    
    # 计算最终坐标 (在XY平面内的扇形)
    FINAL_X=$(python3 -c "import math; print(f'{$CENTER_X + $RANDOM_R * math.cos($RANDOM_THETA):.3f}')")
    FINAL_Y=$(python3 -c "import math; print(f'{$CENTER_Y + $RANDOM_R * math.sin($RANDOM_THETA):.3f}')")
    FINAL_Z=$CENTER_Z  # Z坐标保持不变

    echo "随机参数: 角度=$RANDOM_THETA, 半径=$RANDOM_R"

else
    echo "错误: 无效的模式 $MODE，必须是0或1"
    exit 1
fi
echo "最终目标点: ($FINAL_X, $FINAL_Y, $FINAL_Z)"
echo "=========================================="


# 发送目标点
echo "正在发送目标点..."
python3 send_single_goal.py $FINAL_X $FINAL_Y $FINAL_Z $ROLL $PITCH $YAW


echo "=========================================="
echo "目标点发送完成!"
echo "=========================================="


