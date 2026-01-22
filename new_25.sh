#!/bin/bash

# 每个任务的文件数量倍数参数
FILES_PER_TASK=1

# 设置工作目录（根据实际情况修改）
WORKSPACE_DIR="$HOME/Desktop/DataCollect"

# 日志文件路径
LOG_DIR="$WORKSPACE_DIR/logs"
LOG_FILE="$LOG_DIR/25_$(date +%Y%m%d_%H%M%S).log"

# 创建日志目录
mkdir -p "$LOG_DIR"

# 设置环境变量（根据实际情况修改）
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1

# 记录开始时间
echo "===== EGO Runner 启动 =====" | tee -a "$LOG_FILE"
echo "开始时间: $(date)" | tee -a "$LOG_FILE"

# 主循环 - 从all_task.txt中读取任务并执行
ALL_TASK_FILE="$WORKSPACE_DIR/all_task.txt"
TASK_FILE="$WORKSPACE_DIR/task.txt"
DELETE_INDEX_FILE="$WORKSPACE_DIR/delete_index.txt"

# 检查all_task.txt文件是否存在
if [ ! -f "$ALL_TASK_FILE" ]; then
    echo "错误: $ALL_TASK_FILE 文件不存在！" | tee -a "$LOG_FILE"
    exit 1
fi

# 计算all_task.txt中的总行数
TOTAL_LINES=$(wc -l < "$ALL_TASK_FILE")
echo "all_task.txt 总行数: $TOTAL_LINES" | tee -a "$LOG_FILE"

# 解析任务数据，计算任务组数
echo "开始解析任务数据格式..." | tee -a "$LOG_FILE"
TASK_GROUPS=0
CURRENT_LINE=1

# 预扫描计算任务组数
while [ $CURRENT_LINE -le $TOTAL_LINES ]; do
    # 读取当前行内容
    LINE_CONTENT=$(sed -n "${CURRENT_LINE}p" "$ALL_TASK_FILE")
    
    # 跳过空行
    if [ -z "$LINE_CONTENT" ]; then
        echo "跳过第 $CURRENT_LINE 行（空行）" | tee -a "$LOG_FILE"
        CURRENT_LINE=$((CURRENT_LINE + 1))
        continue
    fi
    
    # 检查是否为数字
    if [[ "$LINE_CONTENT" =~ ^[0-9]+$ ]]; then
        DATA_LINES="$LINE_CONTENT"
        TASK_GROUPS=$((TASK_GROUPS + 1))
        echo "找到任务组 $TASK_GROUPS: 包含 $DATA_LINES 行数据（从第 $CURRENT_LINE 行开始）" | tee -a "$LOG_FILE"
        CURRENT_LINE=$((CURRENT_LINE + DATA_LINES + 1))  # 跳过数字行和数据行
    else
        echo "警告: 第 $CURRENT_LINE 行不是有效的数字: $LINE_CONTENT" | tee -a "$LOG_FILE"
        CURRENT_LINE=$((CURRENT_LINE + 1))
    fi
done

echo "总任务组数: $TASK_GROUPS" | tee -a "$LOG_FILE"

# 重置行号开始处理任务
CURRENT_LINE=1

# 循环处理每个任务组
current_group=0
while [ $current_group -lt $TASK_GROUPS ] && [ $CURRENT_LINE -le $TOTAL_LINES ]; do
    # 跳过空行
    LINE_CONTENT=$(sed -n "${CURRENT_LINE}p" "$ALL_TASK_FILE")
    if [ -z "$LINE_CONTENT" ]; then
        echo "跳过第 $CURRENT_LINE 行（空行）" | tee -a "$LOG_FILE"
        CURRENT_LINE=$((CURRENT_LINE + 1))
        continue
    fi
    
    # 检查是否为数字（任务组开始）
    if [[ "$LINE_CONTENT" =~ ^[0-9]+$ ]]; then
        current_group=$((current_group + 1))
        DATA_LINES="$LINE_CONTENT"

        DELETE_INDEX=0
        if [ -f "$DELETE_INDEX_FILE" ]; then
            DELETE_INDEX=$(sed -n "${current_group}p" "$DELETE_INDEX_FILE")
            if [ -z "$DELETE_INDEX" ]; then
                DELETE_INDEX=0
            fi
            if ! [[ "$DELETE_INDEX" =~ ^[0-9]+$ ]]; then
                echo "警告: $DELETE_INDEX_FILE 第 $current_group 行不是有效数字，忽略" | tee -a "$LOG_FILE"
                DELETE_INDEX=0
            fi
        else
            echo "警告: $DELETE_INDEX_FILE 文件不存在，将不触发隐藏逻辑" | tee -a "$LOG_FILE"
        fi
        echo "任务组 $current_group 隐藏触发轮次: $DELETE_INDEX" | tee -a "$LOG_FILE"
        
        echo -e "\n[$(date)] 处理第 $current_group/$TASK_GROUPS 个任务组" | tee -a "$LOG_FILE"
        echo "当前任务组包含 $DATA_LINES 行数据" | tee -a "$LOG_FILE"
        
        # 提取整个数据块（包括数字行）到task.txt
        END_LINE=$((CURRENT_LINE + DATA_LINES))
        sed -n "${CURRENT_LINE},${END_LINE}p" "$ALL_TASK_FILE" > "$TASK_FILE"
        
        echo "已将第 $CURRENT_LINE-$END_LINE 行数据写入 task.txt" | tee -a "$LOG_FILE"
        echo "task.txt 内容:" | tee -a "$LOG_FILE"
        cat "$TASK_FILE" | tee -a "$LOG_FILE"
    
    
    # 计算目标文件数量：FILES_PER_TASK次 × (DATA_LINES-1)个任务
    TARGET_FILES=$((FILES_PER_TASK * (DATA_LINES-1)))
    echo "任务组 $current_group 目标文件数: $TARGET_FILES (每个任务 $FILES_PER_TASK 个文件)" | tee -a "$LOG_FILE"
    
    # 循环执行直到收集足够的文件
    cycle=1
    while true; do
        # 检查save_data目录中的文件夹数量（仅第一层）
        SAVE_DATA_DIR="$WORKSPACE_DIR/save_data"
        if [ -d "$SAVE_DATA_DIR" ]; then
            CURRENT_FILES=$(find "$SAVE_DATA_DIR" -maxdepth 1 -type d ! -path "$SAVE_DATA_DIR" | wc -l)
        else
            CURRENT_FILES=0
        fi
        
        echo -e "\n[$(date)] 任务组 $current_group - 第 $cycle 次执行" | tee -a "$LOG_FILE"
        echo "当前save_data中文件数: $CURRENT_FILES / $TARGET_FILES" | tee -a "$LOG_FILE"
        
        # 如果已经收集够了文件，删除多余的文件夹后跳出循环
        if [ $CURRENT_FILES -ge $TARGET_FILES ]; then
            echo "已收集到 $CURRENT_FILES 个文件，目标为 $TARGET_FILES 个" | tee -a "$LOG_FILE"
            
            # 如果文件数量超过目标，删除多余的文件夹
            if [ $CURRENT_FILES -gt $TARGET_FILES ]; then
                EXCESS_FILES=$((CURRENT_FILES - TARGET_FILES))
                echo "文件数量超过目标 $EXCESS_FILES 个，开始删除多余文件夹..." | tee -a "$LOG_FILE"
                
                # 获取save_data中按数字排序的文件夹列表，删除最后的多余文件夹
                cd "$SAVE_DATA_DIR" || exit 1
                FOLDER_LIST=($(ls -1d */ 2>/dev/null | sed 's/\///g' | grep '^[0-9]\+$' | sort -n))
                
                # 从最大编号开始删除多余的文件夹
                for ((i=0; i<EXCESS_FILES; i++)); do
                    if [ ${#FOLDER_LIST[@]} -gt 0 ]; then
                        LAST_INDEX=$((${#FOLDER_LIST[@]} - 1 - i))
                        if [ $LAST_INDEX -ge 0 ]; then
                            FOLDER_TO_DELETE="${FOLDER_LIST[$LAST_INDEX]}"
                            echo "删除多余文件夹: $FOLDER_TO_DELETE" | tee -a "$LOG_FILE"
                            rm -rf "$FOLDER_TO_DELETE"
                        fi
                    fi
                done
                
                # 重新计算文件数量
                CURRENT_FILES=$(find "$SAVE_DATA_DIR" -maxdepth 1 -type d ! -path "$SAVE_DATA_DIR" | wc -l)
                echo "删除后剩余文件数: $CURRENT_FILES" | tee -a "$LOG_FILE"
                cd "$WORKSPACE_DIR" || exit 1
            fi
            
            echo "已收集足够文件，跳出执行循环" | tee -a "$LOG_FILE"
            break
        fi
        
        # 进入工作目录并启动程序
        (
            cd "$WORKSPACE_DIR" || exit 1
            if [ -f devel/setup.bash ]; then
                source devel/setup.bash
            else
                echo "警告: devel/setup.bash 不存在，跳过环境设置" | tee -a "$LOG_FILE"
            fi
            ./new_all.sh "Delete${current_group}" "$DELETE_INDEX"
        ) | tee -a "$LOG_FILE" 2>&1
        
        # 记录结束状态
        EXIT_CODE=$?
        if [ $EXIT_CODE -eq 0 ]; then
            echo "[$(date)] 第 $cycle 次执行正常退出 (状态码: $EXIT_CODE)" | tee -a "$LOG_FILE"
        else
            echo "[$(date)] 警告: 第 $cycle 次执行异常退出 (状态码: $EXIT_CODE)" | tee -a "$LOG_FILE"
        fi

        # 等待1秒后执行下一次
        echo "[$(date)] 等待1秒后检查文件数量..." | tee -a "$LOG_FILE"
        sleep 1
        
        cycle=$((cycle + 1))

        # 安全措施：防止无限循环，最多执行125次
        if [ $cycle -gt 125 ]; then
            echo "警告: 已执行125次，强制退出循环" | tee -a "$LOG_FILE"
            break
        fi
    done
        
        echo "[$(date)] 任务组 $current_group 的执行已完成，共执行了 $((cycle-1)) 次" | tee -a "$LOG_FILE"
        ./organize_save_data.sh $((DATA_LINES-1))
        
        # 读取prompts.txt中对应任务组的指令并保存到all_data1的最新文件夹
        PROMPTS_FILE="$WORKSPACE_DIR/prompts.txt"
        ALL_DATA1_DIR="$WORKSPACE_DIR/all_data2"

        if [ -f "$PROMPTS_FILE" ]; then
            # 读取当前任务组对应的指令行（第current_group行）
            INSTRUCTION=$(sed -n "${current_group}p" "$PROMPTS_FILE")
            
            if [ -n "$INSTRUCTION" ]; then
                # 找到all_data1中最新的文件夹
                if [ -d "$ALL_DATA1_DIR" ]; then
                    LATEST_FOLDER=$(ls -1d "$ALL_DATA1_DIR"/*/ 2>/dev/null | sort -V | tail -1)
                    
                    if [ -n "$LATEST_FOLDER" ]; then
                        # 将指令保存到最新文件夹中的instruction.txt文件
                        echo "$INSTRUCTION" > "${LATEST_FOLDER}instruction.txt"
                        echo "[$(date)] 已将指令保存到 ${LATEST_FOLDER}instruction.txt: $INSTRUCTION" | tee -a "$LOG_FILE"
                    else
                        echo "[$(date)] 警告: all_data1目录中没有找到文件夹" | tee -a "$LOG_FILE"
                    fi
                else
                    echo "[$(date)] 警告: all_data1目录不存在" | tee -a "$LOG_FILE"
                fi
            else
                echo "[$(date)] 警告: prompts.txt中第 $current_group 行为空" | tee -a "$LOG_FILE"
            fi
        else
            echo "[$(date)] 警告: prompts.txt文件不存在" | tee -a "$LOG_FILE"
        fi
        
        ./delete.sh
        
        # 更新当前行号到下一个任务组
        CURRENT_LINE=$((CURRENT_LINE + DATA_LINES + 1))
        
        # 如果不是最后一个任务组，等待一段时间
        if [ $current_group -lt $TASK_GROUPS ]; then
            echo "[$(date)] 等待0.5秒后处理下一个任务组..." | tee -a "$LOG_FILE"
            sleep 0.5
        fi
    else
        # 如果不是数字行，跳过
        CURRENT_LINE=$((CURRENT_LINE + 1))
    fi
done

echo -e "\n[$(date)] 所有任务组处理完成！" | tee -a "$LOG_FILE"
