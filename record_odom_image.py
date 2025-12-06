#!/usr/bin/env python3
import rospy
import time
import sys
import os
import threading
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import message_filters
import collections

class PositionImageRecorder:
    def __init__(self, duration, output_dir, image_dir, data_file):
        self.recording = False
        self.start_time = 0
        self.duration = float(duration)
        self.output_dir = output_dir
        self.image_dir = image_dir
        self.depth_dir = os.path.join(os.path.dirname(image_dir), "depth_images")
        self.data_file = data_file
        self.target_file = os.path.join(output_dir, "target_points.csv")
        self.file = None
        self.bridge = CvBridge()
        self.target_position = None
        self.position_tolerance = 0.2
        self.orientation_tolerance = 0.1
        self.target_counter = 0
        self.reached_target = False
        self.extra_frames_count = 0
        self.max_extra_frames = 6
        
        # 图像计数器
        self.image_counters = {
            'camera0': 0, 'camera1': 0, 'camera2': 0, 'camera3': 0
        }
        self.depth_counters = {
            'camera0': 0, 'camera1': 0, 'camera2': 0, 'camera3': 0
        }
        
        # 确保目录存在
        os.makedirs(self.image_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        
        # 创建摄像头目录
        self.camera_dirs = {
            'camera0': os.path.join(self.image_dir, 'front'),
            'camera1': os.path.join(self.image_dir, 'back'),
            'camera2': os.path.join(self.image_dir, 'left'),
            'camera3': os.path.join(self.image_dir, 'right')
        }
        self.depth_camera_dirs = {
            'camera0': os.path.join(self.depth_dir, 'front'),
            'camera1': os.path.join(self.depth_dir, 'back'),
            'camera2': os.path.join(self.depth_dir, 'left'),
            'camera3': os.path.join(self.depth_dir, 'right')
        }
        
        for camera_dir in self.camera_dirs.values():
            os.makedirs(camera_dir, exist_ok=True)
        for depth_camera_dir in self.depth_camera_dirs.values():
            os.makedirs(depth_camera_dir, exist_ok=True)
        
        # 初始化目标点文件
        with open(self.target_file, 'w') as f:
            f.write("目标点ID,时间戳(秒),位置X,位置Y,位置Z,姿态X,姿态Y,姿态Z,姿态W\n")
        
        # 订阅目标点话题
        rospy.Subscriber("/uav_actions", PoseStamped, self.goal_callback)

        # ADD
        self.reached_pub = rospy.Publisher('/target_reached', Bool, queue_size=10)
        self.reached_status = False
        
        # 使用message_filters同步订阅
        # 创建同步器列表
        self.syncers = {}
        
        # 为每个摄像头创建同步器
        for cam_id in ['camera0', 'camera1', 'camera2', 'camera3']:
            # 订阅彩色图像
            color_sub = message_filters.Subscriber(f'/{cam_id}/color/image/compressed', CompressedImage)
            # 订阅深度图像
            depth_sub = message_filters.Subscriber(f'/{cam_id}/depth/image/compressed', CompressedImage)
            # 订阅里程计
            odom_sub = message_filters.Subscriber('/unity_depth_odom0', Odometry)
            
            # 创建近似时间同步器
            ts = message_filters.ApproximateTimeSynchronizer(
                [color_sub, depth_sub, odom_sub],
                queue_size=10,
                slop=0.1,  # 时间容差0.1秒
                allow_headerless=True
            )
            
            # 注册回调函数，传递摄像头ID
            ts.registerCallback(lambda msg1, msg2, msg3, cam_id=cam_id: 
                              self.sync_callback(msg1, msg2, msg3, cam_id))
            
            self.syncers[cam_id] = ts
        
        rospy.loginfo("位置和图像记录器已初始化，等待目标点...")
        print(f"记录编号: {os.path.basename(output_dir)}")
        print("位置和图像记录器已初始化，等待目标点...")
        print(f"目标点将存储在: {self.target_file}")
        print("摄像头图像将存储在:")
        for cam_id, cam_dir in self.camera_dirs.items():
            cam_name = {'camera0': '前', 'camera1': '后', 'camera2': '左', 'camera3': '右'}[cam_id]
            print(f"  {cam_name}摄像头: {os.path.basename(cam_dir)}/")
        print("深度图像将存储在:")
        for cam_id, depth_dir in self.depth_camera_dirs.items():
            cam_name = {'camera0': '前', 'camera1': '后', 'camera2': '左', 'camera3': '右'}[cam_id]
            print(f"  {cam_name}深度摄像头: {os.path.basename(depth_dir)}/")
    
    def goal_callback(self, msg):
        current_time = time.time() - self.start_time if self.recording else 0.0
        with open(self.target_file, 'a') as f:
            pos = msg.pose.position
            ori = msg.pose.orientation
            f.write(f"{self.target_counter},{current_time:.3f},")
            f.write(f"{pos.x:.3f},{pos.y:.3f},{pos.z:.3f},")
            f.write(f"{ori.x:.6f},{ori.y:.6f},{ori.z:.6f},{ori.w:.6f}\n")
        
        self.target_counter += 1
        print(f"✓ 已保存目标点 #{self.target_counter} 到 {os.path.basename(self.target_file)}")
        print(f"目标点位置: X={pos.x:.2f}, Y={pos.y:.2f}, Z={pos.z:.2f}")
        
        if not self.recording:
            self.recording = True
            self.start_time = time.time()
            self.file = open(self.data_file, 'w')
            self.file.write("时间戳(秒),位置X,位置Y,位置Z,姿态X,姿态Y,姿态Z,姿态W,前摄像头图像,前深度图像\n")
            
            self.target_position = {
                'position': msg.pose.position,
                'orientation': msg.pose.orientation
            }
            
            rospy.loginfo("目标点已接收! 开始记录位置数据和图像...")
            print("✓ 目标点已接收! 开始记录位置数据和图像...")

        self.target_position = {
            'position': msg.pose.position,
            'orientation': self.target_position['orientation']
        }
    
    def check_position_reached(self, position, orientation):
        if self.target_position is None:
            return False
        
        pos_diff = np.sqrt(
            (position.x - self.target_position['position'].x) ** 2 +
            (position.y - self.target_position['position'].y) ** 2 +
            (position.z - self.target_position['position'].z) ** 2
        )
        
        def quaternion_to_yaw(q):
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            return np.arctan2(siny_cosp, cosy_cosp)
        
        target_ori = self.target_position['orientation']
        current_yaw = quaternion_to_yaw(orientation)
        target_yaw = quaternion_to_yaw(target_ori)
        
        yaw_diff = current_yaw - target_yaw
        while yaw_diff > np.pi:
            yaw_diff -= 2 * np.pi
        while yaw_diff < -np.pi:
            yaw_diff += 2 * np.pi
        yaw_diff = abs(yaw_diff)
        
        position_reached = pos_diff < self.position_tolerance
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print(pos_diff)
        orientation_reached = yaw_diff < self.orientation_tolerance
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print(f"yaw:diff:{yaw_diff}, current_yaw:{current_yaw}, target_yaw:{target_yaw}")
        if position_reached:
            self.reached_status = True
            reached_msg = Bool()
            reached_msg.data = True
            self.reached_pub.publish(reached_msg)
            print("✓ 已发布目标点到达消息: True")
        
        return position_reached and orientation_reached
    
    def sync_callback(self, color_msg, depth_msg, odom_msg, camera_id):
        if not self.recording:
            return
        
        # 检查时间同步性
        color_time = color_msg.header.stamp.to_sec() if color_msg.header.stamp else 0
        depth_time = depth_msg.header.stamp.to_sec() if depth_msg.header.stamp else 0
        odom_time = odom_msg.header.stamp.to_sec() if odom_msg.header.stamp else 0
        
        # 计算最大时间差
        times = [color_time, depth_time, odom_time]
        max_time_diff = max(times) - min(times) if times else 0
        
        if max_time_diff > 0.2:  # 如果时间差超过200ms，警告
            print(f"⚠️ 时间同步警告: 摄像头 {camera_id} 的消息时间差 {max_time_diff:.3f}s")
        
        try:
            # 处理彩色图像
            cv_color = self.bridge.compressed_imgmsg_to_cv2(color_msg, "bgr8")
            color_filename = f"{camera_id}_{self.image_counters[camera_id]:05d}.jpg"
            color_path = os.path.join(self.camera_dirs[camera_id], color_filename)
            cv2.imwrite(color_path, cv_color)
            self.image_counters[camera_id] += 1
            
            # 处理深度图像
            cv_depth = self.bridge.compressed_imgmsg_to_cv2(depth_msg, "passthrough")
            depth_filename = f"depth_{camera_id}_{self.depth_counters[camera_id]:05d}.png"
            depth_path = os.path.join(self.depth_camera_dirs[camera_id], depth_filename)
            cv2.imwrite(depth_path, cv_depth)
            self.depth_counters[camera_id] += 1
            
            # 摄像头名称映射
            cam_names = {'camera0': '前', 'camera1': '后', 'camera2': '左', 'camera3': '右'}
            cam_name = cam_names[camera_id]
            
            print(f"✓ 已同步保存{cam_name}摄像头: {color_filename}, 深度: {depth_filename}")
            
            # 如果是前摄像头，记录到CSV文件
            if camera_id == 'camera0':
                current_time = odom_time - self.start_time
                
                # 检查是否到达目标点
                if self.target_position is not None and not self.reached_target:
                    position_reached = self.check_position_reached(
                        odom_msg.pose.pose.position,
                        odom_msg.pose.pose.orientation
                    )
                    if position_reached:
                        print("✓ 已到达目标点! 将额外记录2帧后停止...")
                        self.reached_target = True
                        # # 添加：发布到达消息
                        # self.reached_status = True
                        # reached_msg = Bool()
                        # reached_msg.data = True
                        # self.reached_pub.publish(reached_msg)
                        # print("✓ 已发布目标点到达消息: True")
                
                # 记录数据到CSV
                pos = odom_msg.pose.pose.position
                ori = odom_msg.pose.pose.orientation
                self.file.write(f"{current_time:.3f},{pos.x:.3f},{pos.y:.3f},{pos.z:.3f},")
                self.file.write(f"{ori.x:.6f},{ori.y:.6f},{ori.z:.6f},{ori.w:.6f},")
                self.file.write(f"{color_filename},{depth_filename}\n")
                
                # 检查额外记录逻辑
                if self.reached_target:
                    self.extra_frames_count += 1
                    print(f"✓ 额外记录第 {self.extra_frames_count} 帧")
                    
                    if self.extra_frames_count >= self.max_extra_frames:
                        print(f"✓ 已完成额外记录 {self.max_extra_frames} 帧! 将停止记录...")
                        self.stop_recording(f"已到达目标点并完成额外记录{self.max_extra_frames}帧")
            
        except Exception as e:
            rospy.logerr(f"处理摄像头 {camera_id} 数据时出错: {str(e)}")
    
    def stop_recording(self, reason="未知原因"):
        if self.recording:
            if self.reached_status:
                reached_msg = Bool()
                reached_msg.data = False  # 记录结束时重置为False
                self.reached_pub.publish(reached_msg)
                print("✓ 已发布目标点到达消息: False (记录结束)")
            self.recording = False
            if self.file:
                self.file.close()
            rospy.loginfo(f"记录完成! ({reason}) 数据保存在 {self.output_dir}")
            print(f"✓ 记录完成! ({reason})")
            print(f"位置数据: {os.path.basename(self.data_file)}")
            print(f"目标点数据: {os.path.basename(self.target_file)}")
            
            print("图像统计:")
            total_color = 0
            for cam_id, count in self.image_counters.items():
                cam_name = {'camera0': '前', 'camera1': '后', 'camera2': '左', 'camera3': '右'}[cam_id]
                print(f"  {cam_name}摄像头: {count} 张")
                total_color += count
            print(f"  彩色图像总计: {total_color} 张")
            
            print("深度图像统计:")
            total_depth = 0
            for cam_id, count in self.depth_counters.items():
                cam_name = {'camera0': '前', 'camera1': '后', 'camera2': '左', 'camera3': '右'}[cam_id]
                print(f"  {cam_name}深度摄像头: {count} 张")
                total_depth += count
            print(f"  深度图像总计: {total_depth} 张")
            print(f"  所有图像总计: {total_color + total_depth} 张")
            rospy.signal_shutdown(reason)

if __name__ == '__main__':
    if len(sys.argv) < 5:
        print("参数错误: 需要记录时长、输出目录、图像目录和数据文件")
        print("用法: rosrun your_package position_image_recorder.py <duration> <output_dir> <image_dir> <data_file>")
        sys.exit(1)
        
    rospy.init_node('position_image_recorder', anonymous=True)
    recorder = PositionImageRecorder(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        if recorder.file:
            recorder.file.close()
        print("脚本被中断! 已保存数据.")
