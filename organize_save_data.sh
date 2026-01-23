#!/bin/bash

# 脚本：将save_data里的文件按指定数量分组存储在新的文件夹里
# 新文件夹命名为 1 2 3 ...，存储的文件夹重命名为 1-1 1-2 1-3, 2-1 2-2 2-3 等

# 可配置参数
FILES_PER_GROUP=1  # 默认每组包含的文件夹数量

# 如果命令行有参数，使用命令行参数作为每组文件夹数量
if [ $# -eq 1 ]; then
    if [[ $1 =~ ^[0-9]+$ ]] && [ $1 -gt 0 ]; then
        FILES_PER_GROUP=$1
        echo "使用命令行参数：每组 $FILES_PER_GROUP 个文件夹"
    else
        echo "错误：请提供有效的正整数作为每组文件夹数量"
        echo "用法：$0 [每组文件夹数量]"
        echo "例如：$0 2  # 每组2个文件夹"
        echo "例如：$0 4  # 每组4个文件夹"
        exit 1
    fi
else
    echo "使用默认设置：每组 $FILES_PER_GROUP 个文件夹"
    echo "提示：你也可以使用 '$0 数字' 来指定每组文件夹数量"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
TARGET_BASE_DIR="$WORKSPACE_DIR/all_data2"


# 查找现有记录的最大编号
LAST_NUM=0
for dir in "$TARGET_BASE_DIR"/*; do
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
TARGET_BASE_DIR="$TARGET_BASE_DIR/$NEW_NUM"
mkdir -p "$TARGET_BASE_DIR"

SOURCE_DIR="$WORKSPACE_DIR/save_data"

# 获取所有数字文件夹并排序
folders=($(ls -1 "$SOURCE_DIR" | grep '^[0-9]\+$' | sort -n))

echo "找到 ${#folders[@]} 个文件夹需要重新组织"

# 计算需要创建多少个组文件夹
total_folders=${#folders[@]}
groups=$(((total_folders + FILES_PER_GROUP - 1) / FILES_PER_GROUP))  # 向上取整

echo "将创建 $groups 个组文件夹，每组最多 $FILES_PER_GROUP 个文件夹"

# 处理每组文件夹
for ((group=1; group<=groups; group++)); do
    group_dir="$TARGET_BASE_DIR/$group"
    mkdir -p "$group_dir"
    echo "创建组文件夹: $group"
    
    # 处理当前组的文件夹
    for ((i=1; i<=FILES_PER_GROUP; i++)); do
        folder_index=$((($group-1) * FILES_PER_GROUP + $i - 1))
        
        # 检查是否还有文件夹需要处理
        if [ $folder_index -lt $total_folders ]; then
            source_folder="${folders[$folder_index]}"
            target_folder="$group_dir/$group-$i"
            
            echo "  复制文件夹 $source_folder -> $group-$i"
            
            # 复制文件夹及其内容
            cp -r "$SOURCE_DIR/$source_folder" "$target_folder"
            
            if [ $? -eq 0 ]; then
                echo "    成功复制: $source_folder -> $group-$i"
            else
                echo "    错误: 复制 $source_folder 失败"
            fi
        fi
    done
    echo "组 $group 处理完成"
    echo "---"
done

echo "所有文件夹重新组织完成！"
echo "原始文件在: $SOURCE_DIR"
echo "重新组织的文件在: $TARGET_BASE_DIR"

# 显示结果统计
echo ""
echo "结果统计:"
total_copied=0
for ((group=1; group<=groups; group++)); do
    group_dir="$TARGET_BASE_DIR/$group"
    if [ -d "$group_dir" ]; then
        count=$(ls -1 "$group_dir" | wc -l)
        echo "  组 $group: $count 个文件夹"
        total_copied=$((total_copied + count))
    fi
done
echo "  总共复制了 $total_copied 个文件夹"
