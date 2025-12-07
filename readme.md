1. 安装必要的依赖
```bash
conda deactivate

sudo apt update
sudo apt upgrade

sudo apt install python3-empy
sudo apt install libglfw3-dev
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
sudo apt install libv4l-dev v4l-utils
sudo apt install ros-noetic-apriltag
sudo apt install ros-noetic-apriltag-ros

sudo ln -s /usr/bin/python3 /usr/bin/python

sudo apt install terminator

```
2. 编译`EGO-Planner-v3`,`Fast-Perching`,`lidar_filter`和`ROS-Unity_bridge`
```
cd /path/to/xxx
catkin_make
```
3. 运行流程。

人工标注的数据放于`3.xlsx`内，然后运行
```bash
python3 process.py
```
任务会存放到`all_task.txt`中。
然后运行unity的场景。
接着在黑色的终端（上面安装的terminator里）运行
```
./new_25.sh
```
和
```
./filter.bash
```
即可开始数据采集。

4. 可视化采集流程
```
./rviz.bash
```