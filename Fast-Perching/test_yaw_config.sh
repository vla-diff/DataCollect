#!/bin/bash

# Fast-Perching Yaw 测试脚本
# 该脚本用于测试不同的yaw配置

echo "=== Fast-Perching Yaw配置测试脚本 ==="
echo ""

# 函数：修改launch文件中的参数
modify_launch_param() {
    local param_name=$1
    local param_value=$2
    local launch_file="/home/bozhi/Desktop/DataCollect/UnityVisExample/Fast-Perching/src/planning/launch/perching_unity.launch"
    
    # 使用sed替换参数值
    sed -i "s/<param name=\"${param_name}\" value=\"[^\"]*\" \/>/<param name=\"${param_name}\" value=\"${param_value}\" \/>/" "$launch_file"
    echo "设置 ${param_name} = ${param_value}"
}

echo "请选择yaw配置模式："
echo "1) 固定yaw角度 (90度)"
echo "2) 固定yaw角度 (45度)"  
echo "3) 基于速度方向的yaw"
echo "4) 时变yaw (正弦变化)"
echo "5) 恢复默认配置"
echo ""

read -p "请输入选项 (1-5): " choice

case $choice in
    1)
        echo "配置固定90度yaw..."
        modify_launch_param "fixed_yaw" "1.57"
        modify_launch_param "use_velocity_yaw" "false"
        modify_launch_param "use_time_varying_yaw" "false"
        ;;
    2)
        echo "配置固定45度yaw..."
        modify_launch_param "fixed_yaw" "0.785"
        modify_launch_param "use_velocity_yaw" "false"
        modify_launch_param "use_time_varying_yaw" "false"
        ;;
    3)
        echo "配置基于速度方向的yaw..."
        modify_launch_param "use_velocity_yaw" "true"
        modify_launch_param "yaw_offset" "0.0"
        modify_launch_param "use_time_varying_yaw" "false"
        ;;
    4)
        echo "配置时变yaw (0.5Hz, 30度幅度)..."
        modify_launch_param "fixed_yaw" "0.0"
        modify_launch_param "use_velocity_yaw" "false"
        modify_launch_param "use_time_varying_yaw" "true"
        modify_launch_param "yaw_frequency" "0.5"
        modify_launch_param "yaw_amplitude" "0.524"
        ;;
    5)
        echo "恢复默认配置..."
        modify_launch_param "fixed_yaw" "1.57"
        modify_launch_param "use_velocity_yaw" "false"
        modify_launch_param "use_time_varying_yaw" "false"
        modify_launch_param "yaw_frequency" "1.0"
        modify_launch_param "yaw_amplitude" "0.5"
        ;;
    *)
        echo "无效选项！"
        exit 1
        ;;
esac

echo ""
echo "=== 配置完成！==="
echo "现在您可以启动系统："
echo "  roslaunch planning perching_unity.launch"
echo ""
echo "或者查看配置文件："
echo "  cat /home/bozhi/Desktop/DataCollect/UnityVisExample/Fast-Perching/YAW_CONFIGURATION.md"
