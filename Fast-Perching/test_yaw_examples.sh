#!/bin/bash

echo "=== Fast-Perching Yaw 命令行测试示例 ==="
echo ""

echo "现在您可以使用以下命令来测试不同的yaw配置："
echo ""

echo "1. 固定30度yaw角："
echo "   roslaunch planning perching_unity.launch fixed_yaw:=0.524"
echo ""

echo "2. 固定120度yaw角："
echo "   roslaunch planning perching_unity.launch fixed_yaw:=2.094"
echo ""

echo "3. 基于速度方向的yaw："
echo "   roslaunch planning perching_unity.launch use_velocity_yaw:=true"
echo ""

echo "4. 时变yaw (频率0.5Hz，幅度30度)："
echo "   roslaunch planning perching_unity.launch use_time_varying_yaw:=true yaw_frequency:=0.5 yaw_amplitude:=0.524"
echo ""

echo "5. 组合配置 (基础45度 + 时变)："
echo "   roslaunch planning perching_unity.launch fixed_yaw:=0.785 use_time_varying_yaw:=true yaw_amplitude:=0.262"
echo ""

echo "验证yaw输出："
echo "   rostopic echo /drone_0_planning/pos_cmd"
echo ""

echo "查看更详细的配置说明："
echo "   cat /home/bozhi/Desktop/DataCollect/UnityVisExample/Fast-Perching/YAW_CONFIG_GUIDE.md"
