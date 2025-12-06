#!/bin/bash

MODEL_STATE=1 #用于记录需要抓取的物体现在的状态，0是隐藏，1是显示

# 配置参数
max_timeout=25  # 超时时间（秒）

# 主要终端PID文件
TERM_PID_FILE1="/tmp/ego_term.pid"        # EGO规划器终端
TERM_PID_FILE2="/tmp/tcp_term.pid"        # TCP终端
TERM_PID_FILE7="/tmp/perching_term.pid"   # Perching终端
TERM_PID_FILE11="/tmp/change_yaw_term.pid" # Yaw终端

# 任务完成标记文件
SAVE_PERCHING_DONE_FILE="/tmp/save_perching_done"
PUB_COMMAND_DONE_FILE="/tmp/pub_command_done"

# 清理可能存在的旧文件
rm -f /tmp/*_term.pid /tmp/save_*_done /tmp/pub_command_done

# 记录程序启动前save_data目录中的文件夹列表
PROGRAM_START_DIRS="/tmp/program_start_dirs.txt"
if [ -d "save_data" ]; then
    find save_data -maxdepth 1 -type d ! -path save_data | sort > "$PROGRAM_START_DIRS"
else
    touch "$PROGRAM_START_DIRS"  # 创建空文件
fi
echo "已记录程序启动前的文件夹状态"

# 读取task.txt文件配置
if [ ! -f "task.txt" ]; then
    echo "错误: task.txt 文件不存在！"
    exit 1
fi

# 读取任务配置
TASK_COUNT=$(head -n 1 task.txt)
echo "从task.txt读取到任务数量: $TASK_COUNT"

# 读取ego.sh参数（第二行）
EGO_PARAMS=$(sed -n "2p" task.txt)
echo "读取到ego.sh参数: $EGO_PARAMS"

# 从第二行提取最后一个参数作为pub_yaw.sh的参数
YAW_PARAM=$(echo $EGO_PARAMS | awk '{print $NF}')
echo "读取到pub_yaw.sh参数: $YAW_PARAM"

# 读取任务配置到数组（从第三行开始）
declare -a TASK_CONFIGS
line_num=3
for ((i=1; i<=TASK_COUNT-1; i++)); do
    TASK_CONFIGS[$i]=$(sed -n "${line_num}p" task.txt)
    ((line_num++))
done

echo "任务配置读取完成..."

# 启动核心程序
echo "启动核心程序..."

# 启动 ego.sh
echo "启动 EGO-Planner..."
gnome-terminal --title="EGO-Planner" -- bash -c "
  echo \$PPID > $TERM_PID_FILE1;
  echo '正在启动 EGO-Planner 主程序...';
  source devel/setup.bash;
  ./ego.sh $EGO_PARAMS; 
  echo '程序已结束，按回车关闭窗口...';
  read;
  " &
sleep 20

# 启动 tcp.sh
echo "启动 TCP..."
gnome-terminal --title="TCP" -- bash -c "
  echo \$PPID > $TERM_PID_FILE2;
  echo '正在启动 TCP 主程序...';
  source devel/setup.bash;
  ./tcp.sh;
  echo '程序已结束，按回车关闭窗口...';
  read;
  " &
sleep 20

# 启动 pub_yaw.sh
echo "启动 Yaw..."
gnome-terminal --title="Yaw" -- bash -c "
  echo \$PPID > $TERM_PID_FILE11;
  echo '正在启动 pub_yaw.sh 主程序...';
  ./pub_yaw.sh $YAW_PARAM;
  echo '程序已结束，按回车关闭窗口...';
  read;
  " &
sleep 20

# 启动 perching.sh
echo "启动 Perching..."
gnome-terminal --title="Perching" -- bash -c "
  echo \$PPID > $TERM_PID_FILE7;
  echo '正在运行 perching.sh...';
  source devel/setup.bash;
  ./perching.sh;
  echo 'perching.sh 完成，按回车关闭窗口...';
  read;
  " &
sleep 20

echo "开始执行循环任务..."

# 循环启动不同脚本 (从task.txt读取配置，从第三行开始)
for cycle in $(seq 1 $((TASK_COUNT-1))); do
  # 解析当前循环的任务配置
  TASK_CONFIG=(${TASK_CONFIGS[$cycle]})
  TASK_TYPE=${TASK_CONFIG[0]}
  
  # 根据任务类型选择脚本和命令
  if [ "$TASK_TYPE" = "1" ]; then
    SCRIPT_NAME="save.sh"
    SCRIPT_TITLE="Save Data"
    # 获取TASK_CONFIG的最后一个参数用于pub_yaw.sh
    LAST_PARAM="${TASK_CONFIG[${#TASK_CONFIG[@]}-1]}"
    # 提取目标位置坐标用于检查是否到达（去掉任务类型后的参数：模式 x y z ...）
    GOAL_PARAMS=(${TASK_CONFIG[@]:1})
    TARGET_X=${GOAL_PARAMS[1]}  # 第2个参数是x坐标
    TARGET_Y=${GOAL_PARAMS[2]}  # 第3个参数是y坐标  
    TARGET_Z=${GOAL_PARAMS[3]}  # 第4个参数是z坐标
    TARGET_YAW=$LAST_PARAM
    PUB_COMMAND="./pub_goal.sh ${TASK_CONFIG[@]:1}; echo '等待到达目标位置...'; ./check_target_reached.sh $TARGET_X $TARGET_Y $TARGET_Z $TARGET_YAW && echo '已到达目标，开始执行yaw调整'; ./pub_yaw.sh $LAST_PARAM; touch $PUB_COMMAND_DONE_FILE"
    PUB_TITLE="Goal Sender"
  elif [ "$TASK_TYPE" = "3" ]; then
    SCRIPT_NAME="save_yaw.sh"
    SCRIPT_TITLE="Save Yaw"
    PUB_COMMAND="./pub_yaw.sh ${TASK_CONFIG[@]:1}; touch $PUB_COMMAND_DONE_FILE"
    PUB_TITLE="Pub Yaw"
  elif [ "$TASK_TYPE" = "2" ]; then
    SCRIPT_NAME="save_perching.sh"
    SCRIPT_TITLE="Save Perching"
    # 获取参数并将第一个数字减1
    PARAMS=(${TASK_CONFIG[@]:1})
    FIRST_PARAM=$((${PARAMS[0]} - 1))
    # 提取目标位置坐标用于检查是否到达（参数格式：模式 x y z ...）
    TARGET_X=${PARAMS[1]}  # 第2个参数是x坐标
    TARGET_Y=${PARAMS[2]}  # 第3个参数是y坐标  
    TARGET_Z=${PARAMS[3]}  # 第4个参数是z坐标
    TARGET_YAW=0.0  # perching任务通常不涉及特定yaw角度
    PUB_COMMAND="./pub_perching.sh $FIRST_PARAM ${PARAMS[@]:1}; echo '等待到达perching目标位置...'; touch $PUB_COMMAND_DONE_FILE"
    PUB_TITLE="Pub Perching"
  else
    echo "错误: 未知的任务类型 $TASK_TYPE"
    continue
  fi

  echo "第 $cycle 轮：启动 $SCRIPT_NAME 和 $PUB_COMMAND..."

  # 不再需要记录每轮次的状态，因为要删除程序启动后的所有文件夹

  # 清理之前的标记文件
  rm -f $SAVE_PERCHING_DONE_FILE $PUB_COMMAND_DONE_FILE



  # 启动第二个终端（根据轮次运行不同命令）
  echo "启动 $PUB_TITLE 终端 (第 $cycle 轮)..."
  CYCLE_PID2="/tmp/cycle_${cycle}_pub_term.pid"
  gnome-terminal --title="$PUB_TITLE - 轮次 $cycle" -- bash -c "
    echo \$PPID > '$CYCLE_PID2';
    echo '正在运行 $PUB_COMMAND (第 $cycle 轮)...';
    echo '当前工作目录: \$(pwd)';
    if [ -f devel/setup.bash ]; then
      source devel/setup.bash;
    else
      echo '警告: devel/setup.bash 不存在，跳过环境设置';
    fi
    $PUB_COMMAND;
    echo '$PUB_COMMAND 完成 (第 $cycle 轮)，等待2秒后自动关闭...';
    sleep 2;
    " &
  sleep 0.5  # 确保PID文件生成


  # 启动相应的脚本终端
  echo "启动 $SCRIPT_NAME 终端 (第 $cycle 轮)..."
  CYCLE_PID1="/tmp/cycle_${cycle}_save_term.pid"
  gnome-terminal --title="$SCRIPT_TITLE - 轮次 $cycle" -- bash -c "
    echo \$PPID > '$CYCLE_PID1';
    echo '正在运行 $SCRIPT_NAME (第 $cycle 轮)...';
    echo '当前工作目录: \$(pwd)';
    if [ -f devel/setup.bash ]; then
      source devel/setup.bash;
    else
      echo '警告: devel/setup.bash 不存在，跳过环境设置';
    fi
    ./$SCRIPT_NAME;                # 运行相应脚本
    touch $SAVE_PERCHING_DONE_FILE  # 创建完成标记
    echo '$SCRIPT_NAME 完成 (第 $cycle 轮)，等待2秒后关闭...';
    sleep 1;
    " &
  sleep 0.5  # 确保PID文件生成

  # 等待脚本完成，添加超时检测
  echo "等待第 $cycle 轮 $SCRIPT_NAME 和 $PUB_TITLE 运行完成..."
  timeout_counter=0
  task_stuck=false
  
  while [ ! -f $SAVE_PERCHING_DONE_FILE ] || [ ! -f $PUB_COMMAND_DONE_FILE ]; do
      sleep 1
      timeout_counter=$((timeout_counter + 1))
      
      # 显示当前状态
      save_status=""
      pub_status=""
      if [ -f $SAVE_PERCHING_DONE_FILE ]; then
          save_status="✓"
      else
          save_status="⏳"
      fi
      if [ -f $PUB_COMMAND_DONE_FILE ]; then
          pub_status="✓"
      else
          pub_status="⏳"
      fi
      echo "第 $cycle 轮状态: $SCRIPT_NAME $save_status | $PUB_TITLE $pub_status (${timeout_counter}/${max_timeout}s)"
      
      if [ $timeout_counter -ge $max_timeout ]; then
          echo "警告：第 $cycle 轮任务运行超过 ${max_timeout} 秒，判定为卡住状态"
          echo "开始清理程序启动后创建的所有文件夹..."
                    # 强制关闭卡住的终端
          [ -f $CYCLE_PID1 ] && kill -9 $(cat $CYCLE_PID1) 2>/dev/null
          [ -f $CYCLE_PID2 ] && kill -9 $(cat $CYCLE_PID2) 2>/dev/null
          
          # 删除程序启动后创建的所有文件夹
          if [ -d "save_data" ] && [ -f "$PROGRAM_START_DIRS" ]; then
              # 获取当前save_data目录中的文件夹列表
              CURRENT_ALL_DIRS="/tmp/current_all_dirs.txt"
              find save_data -maxdepth 1 -type d ! -path save_data | sort > "$CURRENT_ALL_DIRS"
              
              # 找出程序启动后新增的所有文件夹
              PROGRAM_NEW_DIRS=$(comm -13 "$PROGRAM_START_DIRS" "$CURRENT_ALL_DIRS")
              
              deleted_count=0
              if [ -n "$PROGRAM_NEW_DIRS" ]; then
                  echo "程序启动后创建的所有文件夹："
                  echo "$PROGRAM_NEW_DIRS"
                  
                  # 删除所有符合数字命名规则的新文件夹
                  while IFS= read -r dir; do
                      if [ -d "$dir" ] && [[ "$dir" =~ ^save_data/[0-9]+$ ]]; then
                          echo "删除程序启动后创建的文件夹: $dir"
                          rm -rf "$dir"
                          deleted_count=$((deleted_count + 1))
                      else
                          echo "跳过非数字命名的文件夹: $dir"
                      fi
                  done <<< "$PROGRAM_NEW_DIRS"
              else
                  echo "程序启动后未创建新的文件夹，无需删除"
              fi
              echo "已删除 $deleted_count 个程序启动后创建的文件夹"
              
              # 清理临时文件
              rm -f "$CURRENT_ALL_DIRS"
          else
              echo "警告: save_data 目录不存在或程序启动记录文件丢失"
          fi
        
          
          # 清理PID文件和标记文件
          rm -f $CYCLE_PID1 $CYCLE_PID2 $SAVE_PERCHING_DONE_FILE $PUB_COMMAND_DONE_FILE
          
          echo "由于第 $cycle 轮任务卡住，准备退出循环"
          task_stuck=true
          break
      fi
  done
  
  # 如果任务卡住，跳出for循环
  if [ "$task_stuck" = true ]; then
      echo "由于第 $cycle 轮任务卡住，强制退出for循环"
      bash /home/bozhi/Desktop/DataCollect/HideThing.bash $1 show
      break
  fi
  
  # 只有在正常完成时才进行正常清理
  if [ -f $SAVE_PERCHING_DONE_FILE ] && [ -f $PUB_COMMAND_DONE_FILE ]; then
      echo "第 $cycle 轮 $SCRIPT_NAME 和 $PUB_TITLE 均已完成"
      
    if [ "$MODEL_STATE" -eq 0 ]; then #用于隐藏/显示物体    $MODEL_STATE=0是现在是隐藏，要改成显示   $MODEL_STATE=1是现在是显示，要改成隐藏
        bash ./HideThing.bash $1 show
        MODEL_STATE=1
    else
        bash ./HideThing.bash $1 hide
        MODEL_STATE=0
    fi


      # 清理PID文件
      rm -f $CYCLE_PID1 $CYCLE_PID2 $SAVE_PERCHING_DONE_FILE $PUB_COMMAND_DONE_FILE

      echo "第 $cycle 轮（类型 $TASK_TYPE）完成，等待0.5秒后启动下一轮..."
      sleep 2
  fi
done

# 检查是否是正常完成还是异常退出
if [ $cycle -lt $((TASK_COUNT-1)) ]; then
    echo "任务循环由于异常而提前终止（在第 $cycle 轮）！"
else
    echo "所有任务循环已完成！"
fi

sleep 1  # 确保完全完成

# 关闭所有终端
echo "正在关闭所有程序和终端窗口..."
[ -f $TERM_PID_FILE1 ] && kill -9 $(cat $TERM_PID_FILE1) 2>/dev/null
[ -f $TERM_PID_FILE2 ] && kill -9 $(cat $TERM_PID_FILE2) 2>/dev/null
[ -f $TERM_PID_FILE7 ] && kill -9 $(cat $TERM_PID_FILE7) 2>/dev/null
[ -f $TERM_PID_FILE11 ] && kill -9 $(cat $TERM_PID_FILE11) 2>/dev/null

# 清理临时文件
rm -f /tmp/*_term.pid /tmp/save_*_done /tmp/pub_command_done /tmp/program_start_dirs.txt /tmp/current_all_dirs.txt

echo "所有程序已终止。"