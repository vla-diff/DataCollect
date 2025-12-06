# Fast-Perching Yaw 角度配置指南

本文档说明如何配置无人机的yaw角度，使其以固定不为0的yaw进行规划。

## 修改内容

我们已经修改了轨迹服务器 (`order7_trajectory_server.cpp`) 和启动文件，支持多种yaw角度配置方式。

## 命令行使用方法

### 1. 基本用法

```bash
# 使用默认yaw角度 (perching_unity.launch默认为45度)
roslaunch planning perching_unity.launch

# 使用默认yaw角度 (perching.launch默认为90度)  
roslaunch planning perching.launch
```

### 2. 自定义固定yaw角度

```bash
# 设置固定yaw为30度 (0.524弧度)
roslaunch planning perching_unity.launch fixed_yaw:=0.524

# 设置固定yaw为120度 (2.094弧度)
roslaunch planning perching_unity.launch fixed_yaw:=2.094

# 设置固定yaw为0度 (朝向x轴正方向)
roslaunch planning perching_unity.launch fixed_yaw:=0.0
```

### 3. 基于速度方向的yaw

```bash
# 启用基于速度方向的yaw，无偏移
roslaunch planning perching_unity.launch use_velocity_yaw:=true

# 启用基于速度方向的yaw，并添加90度偏移
roslaunch planning perching_unity.launch use_velocity_yaw:=true yaw_offset:=1.57
```

### 4. 时变yaw (正弦变化)

```bash
# 启用时变yaw，频率0.5Hz，幅度30度
roslaunch planning perching_unity.launch use_time_varying_yaw:=true yaw_frequency:=0.5 yaw_amplitude:=0.524

# 启用时变yaw，基于45度基础，幅度15度变化
roslaunch planning perching_unity.launch fixed_yaw:=0.785 use_time_varying_yaw:=true yaw_amplitude:=0.262
```

### 5. 组合配置示例

```bash
# 复杂配置：基础45度 + 时变yaw
roslaunch planning perching_unity.launch \
    fixed_yaw:=0.785 \
    use_time_varying_yaw:=true \
    yaw_frequency:=1.0 \
    yaw_amplitude:=0.3

# 速度导向yaw + 偏移
roslaunch planning perching_unity.launch \
    use_velocity_yaw:=true \
    yaw_offset:=1.57
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `fixed_yaw` | double | 0.78 (unity) / 1.57 (std) | 固定yaw角度 (弧度制) |
| `use_velocity_yaw` | bool | false | 是否使用基于速度方向的yaw |
| `yaw_offset` | double | 0.0 | 在速度方向基础上的偏移角度 |
| `use_time_varying_yaw` | bool | false | 是否使用时变yaw |
| `yaw_frequency` | double | 1.0 | yaw变化频率 (Hz) |
| `yaw_amplitude` | double | 0.5 | yaw变化幅度 (弧度) |

## 常用角度转换

| 角度(度) | 弧度 | 方向 |
|----------|------|------|
| 0° | 0.0 | X轴正方向 |
| 30° | 0.524 | X-Y象限 |
| 45° | 0.785 | X-Y象限 |
| 90° | 1.57 | Y轴正方向 |
| 120° | 2.094 | Y轴象限 |
| 180° | 3.14 | X轴负方向 |

## 优先级说明

参数的优先级顺序为：
1. `use_time_varying_yaw=true` -> 使用时变yaw
2. `use_velocity_yaw=true` -> 使用速度方向yaw  
3. 否则使用 `fixed_yaw` 固定角度

## 验证方法

启动系统后，可以通过以下方式验证yaw配置：

1. 查看终端输出的配置信息
2. 在rviz中观察无人机模型的朝向
3. 检查发布的 `/drone_0_planning/pos_cmd` 话题中的yaw值

```bash
# 监听position command话题
rostopic echo /drone_0_planning/pos_cmd
```
