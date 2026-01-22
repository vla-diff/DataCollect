import pandas as pd
import numpy as np
import math

df = pd.read_excel('3.xlsx')

# 清理和转换数据
df = df.replace('\\', np.nan)

# 转换数值列
numeric_cols = ['waypoint', 'x', 'y', 'z', 'yaw', 'dx(theta1)', 'dz(theta2)', 'dy(r)', 'random_type', 'Delete']
for col in numeric_cols:
    if col in df.columns:
        df[col] = pd.to_numeric(df[col], errors='coerce')

# 重新组织数据：通过指令文本识别轨迹
trajectories = []
current_trajectory = []

for idx, row in df.iterrows():
    if pd.notna(row['Instruction']):
        if current_trajectory:
            trajectories.append(current_trajectory)
            current_trajectory = []
        current_trajectory.append(row)
    else:
        if current_trajectory and pd.notna(row['waypoint']):
            current_trajectory.append(row)

if current_trajectory:
    trajectories.append(current_trajectory)

print(f"总共提取到 {len(trajectories)} 个轨迹")

# 写入输出文件
with open('all_task.txt', 'w') as f, open('prompts.txt', 'w') as pf, open('delete_index.txt', 'w') as df:
    for i, traj in enumerate(trajectories):
        instruction = traj[0]['Instruction']
        print(f"\n处理轨迹 {i+1}: {instruction}")
        print(f"包含 {len(traj)} 个waypoints")
        
        pf.write(f"轨迹 {i+1}: {instruction}\n")
        f.write(f"{len(traj)}\n")

        delete_indices = [idx for idx, wp in enumerate(traj)
                          if pd.notna(wp.get('Delete')) and int(wp.get('Delete')) == 1]
        if len(delete_indices) > 1:
            print(f"警告: 轨迹 {i+1} 有多个Delete标记，使用第一个: {delete_indices[0]}")
        delete_idx = delete_indices[0] if delete_indices else None
        delete_cycle = delete_idx if delete_idx is not None and delete_idx > 0 else 0
        df.write(f"{delete_cycle}\n")
        
        for wp in traj:
            wp_type = int(wp['waypoint'])

            # ✅ 在写入前计算 yaw
            yaw_ori = wp['yaw']
            yaw = yaw_ori * (-1) * math.pi / 180 if not np.isnan(yaw_ori) else np.nan
            
            if wp_type == 0:
                line = f"{wp['x']}  {wp['z']}  {wp['y']} {wp['dx(theta1)']} {wp['dz(theta2)']} {wp['dy(r)']}  {yaw}"
            elif wp_type == 1:
                line = f"1 {int(wp['random_type'])} {wp['x']}  {wp['z']} {wp['y']} {wp['dx(theta1)']} {wp['dz(theta2)']} {wp['dy(r)']} {yaw}"
            elif wp_type == 2:
                line = f"2 {int(wp['random_type'])} {wp['x']}  {wp['z']} {wp['y']} {wp['dx(theta1)']} {wp['dz(theta2)']} {wp['dy(r)']}"
            elif wp_type == 3:
                line = f"3 {yaw}"
            else:
                print(f"未处理的waypoint类别: {wp_type}")
                continue
            
            f.write(line + "\n")

print(f"\n输出完成，共处理 {len(trajectories)} 个轨迹")
