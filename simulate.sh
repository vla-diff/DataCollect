# 主要终端PID文件
TERM_PID_FILE1="/tmp/ego_term.pid"        # EGO规划器终端
TERM_PID_FILE2="/tmp/tcp_term.pid"        # TCP终端
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
sleep 3

# 启动 tcp.sh
echo "启动 TCP..."
gnome-terminal --title="TCP" -- bash -c "
  echo \$PPID > $TERM_PID_FILE2;
  echo '正在启动 TCP 主程序...';
  source devel/setup.bash;
  ./tcp.sh;
  echo '程序已结束，按回车关闭窗口...';
  read;
  " 

sleep 5
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