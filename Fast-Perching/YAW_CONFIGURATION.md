# 无人机Yaw角度配置说明

本文档说明如何修改无人机的yaw角度规划，使其不再始终为0。

## 修改内容

已经修改了 `/src/planning/src/order7_trajectory_server.cpp` 文件，添加了多种yaw控制模式：

### 1. 固定yaw角度模式（默认）
- 参数: `fixed_yaw`
- 功能: 设置一个固定的yaw角度值
- 单位: 弧度制
- 示例值:
  - `0.0` = 朝向x轴正方向
  - `1.57` = 朝向y轴正方向（90度）
  - `3.14` = 朝向x轴负方向（180度）
  - `-1.57` = 朝向y轴负方向（-90度）

### 2. 基于速度方向的yaw角度
- 参数: `use_velocity_yaw = true`
- 功能: yaw角度跟随无人机的速度方向
- 辅助参数: `yaw_offset` - 在速度方向基础上的偏移量
- 适用场景: 希望无人机"朝向"飞行方向

### 3. 时变yaw角度
- 参数: `use_time_varying_yaw = true`
- 功能: yaw角度按正弦函数变化
- 公式: `yaw = fixed_yaw + amplitude * sin(2*pi*frequency*t)`
- 相关参数:
  - `yaw_frequency`: 变化频率(Hz)
  - `yaw_amplitude`: 变化幅度(弧度)
- 适用场景: 需要无人机在飞行过程中旋转

## 配置示例

### 示例1: 固定90度yaw角度
```xml
<param name="fixed_yaw" value="1.57" />
<param name="use_velocity_yaw" value="false" />
<param name="use_time_varying_yaw" value="false" />
```

### 示例2: 跟随速度方向，偏移45度
```xml
<param name="fixed_yaw" value="0.0" />
<param name="use_velocity_yaw" value="true" />
<param name="yaw_offset" value="0.785" />
<param name="use_time_varying_yaw" value="false" />
```

### 示例3: 基于45度的正弦变化yaw
```xml
<param name="fixed_yaw" value="0.785" />
<param name="use_velocity_yaw" value="false" />
<param name="use_time_varying_yaw" value="true" />
<param name="yaw_frequency" value="0.5" />
<param name="yaw_amplitude" value="0.5" />
```

## 使用方法

1. 编辑launch文件中的参数 (`perching.launch` 或 `perching_unity.launch`)
2. 重新编译项目: `catkin_make`
3. 运行系统: `roslaunch planning perching_unity.launch`

## 注意事项

- 角度使用弧度制，不是角度制
- 如果同时启用多个模式，优先级为: 时变yaw > 速度yaw > 固定yaw
- 时变yaw和速度yaw模式会计算yaw_dot（yaw角速度）
- 建议在仿真中先测试参数效果，再部署到实际系统
