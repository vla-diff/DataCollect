#!/bin/bash

# 无人机位置和图像记录脚本
# 当接收到目标点时开始记录无人机位置、姿态和图像
# 当无人机到达目标点时自动停止记录
# 数据存储在save_data目录下的递增编号文件夹中
# 用法: ./record_odom.sh [记录时长(秒)]

# 设置默认值
RECORD_DURATION=${1:-10}  # 默认记录10秒

# 创建主保存目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
SAVE_DIR="$WORKSPACE_DIR/save_data"
mkdir -p "$SAVE_DIR"

# 查找现有记录的最大编号
LAST_NUM=0
for dir in "$SAVE_DIR"/*; do
    if [[ -d "$dir" ]]; then
        dir_num=${dir##*/}
        if [[ $dir_num =~ ^[0-9]+$ ]]; then
            if (( dir_num > LAST_NUM )); then
                LAST_NUM=$dir_num
            fi
        fi
    fi
done

# 创建新的记录目录
NEW_NUM=$((LAST_NUM + 1))
OUTPUT_DIR="$SAVE_DIR/$NEW_NUM"
mkdir -p "$OUTPUT_DIR"

# 图像保存路径
IMAGE_DIR="$OUTPUT_DIR/images"
mkdir -p "$IMAGE_DIR"

# 数据文件路径
DATA_FILE="$OUTPUT_DIR/data.csv"

echo "=========================================="
echo "  无人机位置和图像记录脚本"
echo "=========================================="
echo "记录时长: $RECORD_DURATION 秒"
echo "记录编号: $NEW_NUM"
echo "输出目录: $OUTPUT_DIR"
echo "图像保存: $IMAGE_DIR"
echo "数据文件: $DATA_FILE"
echo "等待目标点触发..."
echo "=========================================="



# 运行记录脚本
echo "启动位置和图像记录器..."
echo "等待接收目标点触发信号..."
cd "$WORKSPACE_DIR/EGO-Planner-v3"
source "$WORKSPACE_DIR/EGO-Planner-v3/devel/setup.bash"
python3 record_cmd_image.py $RECORD_DURATION "$OUTPUT_DIR" "$IMAGE_DIR" "$DATA_FILE"



echo "=========================================="
echo "位置和图像记录已完成!"
echo "记录编号: $NEW_NUM"
echo "数据目录: $OUTPUT_DIR"
echo "图像数量: $(ls -1 "$IMAGE_DIR" | wc -l)"
echo "记录行数: $(wc -l < "$DATA_FILE")"
echo "=========================================="

# 创建README文件记录参数
cat > "$OUTPUT_DIR/README.txt" << EOL
记录编号: $NEW_NUM
记录时间: $(date)
EOL

echo "已创建README文件: $OUTPUT_DIR/README.txt"
