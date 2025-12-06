#!/bin/bash

# check_target_reached.sh - 持续监听直到收到 true

echo "开始持续监听目标点到达信号..."

# 持续监听，直到收到 true
while true; do
    result=$(rostopic echo -n 1 /target_reached 2>/dev/null | grep "data: ")
    
    if echo "$result" | grep -q "True"; then
        echo "收到目标点到达信号！"
        echo "停止脚本..."
        exit 0
    fi
    
    sleep 0.1  # 短暂休眠避免过度消耗CPU
done