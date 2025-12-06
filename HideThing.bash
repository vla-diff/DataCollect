#!/bin/bash

PROJECT_PATH="/home/bozhi/Downloads/simEnv_2025_8_7/unity/UnityVisExample/UAV_sample"
TARGET_FILE="$PROJECT_PATH/Assets/hide_target.txt"

if [ $# -lt 2 ]; then
    echo "用法: $0 物体名 命令(hide/show)"
    exit 1
fi

OBJECT_NAME=$1
COMMAND=$2

echo "${OBJECT_NAME}:${COMMAND}" > "$TARGET_FILE"
echo "已写入命令 '$COMMAND' 给物体 '$OBJECT_NAME'"
echo "请确保 Unity 已在 Play Mode 运行"
