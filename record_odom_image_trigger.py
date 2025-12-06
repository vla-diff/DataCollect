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
from cv_bridge import CvBridge
import cv2
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber

class PositionImageRecorder:
    def __init__(self, duration, output_dir, image_dir, data_file):
        self.recording = False
        self.start_time = 0
        self.duration = float(duration)
        self.output_dir = output_dir
        self.image_dir = image_dir
        self.depth_dir = os.path.join(os.path.dirname(image_dir), "depth_images")  # 深度图像目录，与image_dir同级
        self.data_file = data_file
        self.target_file = os.path.join(output_dir, "target_points.csv")  # 目标点存储文件
        self.file = None
        self.image_counter = 0
        self.bridge = CvBridge()
        self.latest_odom = None
        self.odom_lock = threading.Lock()
        self.target_position = None  # 存储目标点位置
        self.position_tolerance = 0.1  # 位置容差 (米) - 10厘米
        self.orientation_tolerance = 0.05  # 姿态容差
        self.debug_frame_count = 0  # 调试帧计数
        self.target_counter = 0  # 目标点计数器
        self.reached_target = False  # 标记是否已到达目标点
        self.extra_frames_count = 0  # 额外记录的帧数计数
        self.max_extra_frames = 5  # 最多额外记录的帧数
        
        # 为四个摄像头创建计数器
        self.image_counters = {
            'camera0': 0,  # 前
            'camera1': 0,  # 后
            'camera2': 0,  # 左
            'camera3': 0   # 右
        }
        
        # 为四个深度摄像头创建计数器
        self.depth_counters = {
            'camera0': 0,  # 前
            'camera1': 0,  # 后
            'camera2': 0,  # 左
            'camera3': 0   # 右
        }
        
        # 确保图像目录和目标点文件目录存在
        os.makedirs(self.image_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        
        # 为四个摄像头创建子目录
        self.camera_dirs = {
            'camera0': os.path.join(self.image_dir, 'front'),    # 前
            'camera1': os.path.join(self.image_dir, 'back'),     # 后
            'camera2': os.path.join(self.image_dir, 'left'),     # 左
            'camera3': os.path.join(self.image_dir, 'right')     # 右
        }
        
        # 为四个深度摄像头创建子目录
        self.depth_camera_dirs = {
            'camera0': os.path.join(self.depth_dir, 'front'),    # 前
            'camera1': os.path.join(self.depth_dir, 'back'),     # 后
            'camera2': os.path.join(self.depth_dir, 'left'),     # 左
            'camera3': os.path.join(self.depth_dir, 'right')     # 右
        }
        
        # 创建所有摄像头目录
        for camera_dir in self.camera_dirs.values():
            os.makedirs(camera_dir, exist_ok=True)
            
        # 创建所有深度摄像头目录
        for depth_camera_dir in self.depth_camera_dirs.values():
            os.makedirs(depth_camera_dir, exist_ok=True)
        
        # 初始化目标点文件并写入表头
        with open(self.target_file, 'w') as f:
            f.write("目标点ID,时间戳(秒),位置X,位置Y,位置Z,姿态X,姿态Y,姿态Z,姿态W\n")
        
        # 订阅触发话题
        rospy.Subscriber("/triger", PoseStamped, self.goal_callback)
        
        # 创建消息同步器
        self.setup_synchronizers()
        
        rospy.loginfo("位置和图像记录器已初始化，等待触发信号...")
        print(f"记录编号: {os.path.basename(output_dir)}")
        print("位置和图像记录器已初始化，等待触发信号...")
        print("使用以下命令触发记录并设置目标位置:")
        print('rostopic pub -1 /triger geometry_msgs/PoseStamped "header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: \'\'} pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}"')
        print("注意: 将 x, y, z 替换为您想要的目标位置坐标")
        print(f"目标点将存储在: {self.target_file}")
        print("摄像头图像将存储在:")
        print(f"  前摄像头: {os.path.basename(self.camera_dirs['camera0'])}/")
        print(f"  后摄像头: {os.path.basename(self.camera_dirs['camera1'])}/")
        print(f"  左摄像头: {os.path.basename(self.camera_dirs['camera2'])}/")
        print(f"  右摄像头: {os.path.basename(self.camera_dirs['camera3'])}/")
        print("深度图像将存储在:")
        print(f"  前深度摄像头: {os.path.basename(self.depth_camera_dirs['camera0'])}/")
        print(f"  后深度摄像头: {os.path.basename(self.depth_camera_dirs['camera1'])}/")
        print(f"  左深度摄像头: {os.path.basename(self.depth_camera_dirs['camera2'])}/")
        print(f"  右深度摄像头: {os.path.basename(self.depth_camera_dirs['camera3'])}/")
    
    def setup_synchronizers(self):
        """设置消息同步器，确保所有摄像头的图像、深度和里程计数据时间戳对齐"""
        # 订阅里程计话题（所有摄像头共享）
        odom_sub = Subscriber("/unity_depth_odom0", Odometry)
        
        # 为每个摄像头创建同步器
        camera_topics = {
            'camera0': {
                'rgb': "/camera0/color/image/compressed",
                'depth': "/camera0/depth/image/compressed"
            },
            'camera1': {
                'rgb': "/camera1/color/image/compressed", 
                'depth': "/camera1/depth/image/compressed"
            },
            'camera2': {
                'rgb': "/camera2/color/image/compressed",
                'depth': "/camera2/depth/image/compressed"
            },
            'camera3': {
                'rgb': "/camera3/color/image/compressed",
                'depth': "/camera3/depth/image/compressed"
            }
        }
        
        # 为每个摄像头创建同步器
        self.camera_syncs = {}
        for camera_id, topics in camera_topics.items():
            rgb_sub = Subscriber(topics['rgb'], CompressedImage)
            depth_sub = Subscriber(topics['depth'], CompressedImage)
            
            # 使用近似时间同步器，设置合适的时间容差
            sync = ApproximateTimeSynchronizer(
                [rgb_sub, depth_sub, odom_sub],
                queue_size=10,
                slop=0.1  # 时间容差，单位秒
            )
            
            # 为每个摄像头注册回调函数
            if camera_id == 'camera0':
                sync.registerCallback(lambda rgb, depth, odom: self.camera_sync_callback(rgb, depth, odom, 'camera0'))
            elif camera_id == 'camera1':
                sync.registerCallback(lambda rgb, depth, odom: self.camera_sync_callback(rgb, depth, odom, 'camera1'))
            elif camera_id == 'camera2':
                sync.registerCallback(lambda rgb, depth, odom: self.camera_sync_callback(rgb, depth, odom, 'camera2'))
            elif camera_id == 'camera3':
                sync.registerCallback(lambda rgb, depth, odom: self.camera_sync_callback(rgb, depth, odom, 'camera3'))
            
            self.camera_syncs[camera_id] = sync
    
    def camera_sync_callback(self, rgb_msg, depth_msg, odom_msg, camera_id):
        """摄像头同步回调函数，处理时间戳对齐的RGB、深度和里程计数据"""
        if not self.recording:
            return
            
        # 检查是否到达目标点
        if self.target_position is not None and not self.reached_target:
            position_reached = self.check_position_reached(
                odom_msg.pose.pose.position,
                odom_msg.pose.pose.orientation
            )
            if position_reached:
                print("✓ 已到达目标点! 将额外记录5帧后停止...")
                self.reached_target = True
        
        # 使用同步的时间戳
        msg_time = odom_msg.header.stamp.to_sec()
        
        # 记录开始时的消息时间戳（用于计算相对时间）
        if not hasattr(self, 'start_msg_time'):
            self.start_msg_time = msg_time
        
        current_time = msg_time - self.start_msg_time
        
        # 添加最大记录时间保护（30秒）
        max_recording_time = 30.0
        if current_time > max_recording_time:
            self.stop_recording(f"超过最大记录时间{max_recording_time}秒")
            return
        
        try:
            # 转换压缩图像为OpenCV格式
            cv_image = self.bridge.compressed_imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_depth_image = self.bridge.compressed_imgmsg_to_cv2(depth_msg, "passthrough")
            
            # 生成图像文件名
            image_filename = f"{camera_id}_{self.image_counters[camera_id]:05d}.jpg"
            depth_filename = f"depth_{camera_id}_{self.image_counters[camera_id]:05d}.png"
            
            # 摄像头名称映射
            camera_names = {
                'camera0': '前',
                'camera1': '后', 
                'camera2': '左',
                'camera3': '右'
            }
            
            # 保存RGB图像
            image_path = os.path.join(self.camera_dirs[camera_id], image_filename)
            cv2.imwrite(image_path, cv_image)
            
            # 保存深度图像
            depth_path = os.path.join(self.depth_camera_dirs[camera_id], depth_filename)
            cv2.imwrite(depth_path, cv_depth_image)
            
            # 更新计数器
            self.image_counters[camera_id] += 1
            self.depth_counters[camera_id] += 1
            
            print(f"✓ 已同步保存{camera_names[camera_id]}摄像头: RGB={image_filename}, 深度={depth_filename}")
            
            # 如果是前摄像头，记录到CSV文件
            if camera_id == 'camera0':
                pos = odom_msg.pose.pose.position
                ori = odom_msg.pose.pose.orientation
                
                # 记录数据到CSV
                if self.file is None:
                    self.file = open(self.data_file, 'w')
                    self.file.write("时间戳(秒),位置X,位置Y,位置Z,姿态X,姿态Y,姿态Z,姿态W,前摄像头图像,前深度图像\n")
                
                self.file.write(f"{current_time:.3f},{pos.x:.3f},{pos.y:.3f},{pos.z:.3f},")
                self.file.write(f"{ori.x:.6f},{ori.y:.6f},{ori.z:.6f},{ori.w:.6f},")
                self.file.write(f"{image_filename},{depth_filename}\n")
                
                # 调试信息 - 每10帧打印一次详细信息
                if self.image_counters['camera0'] % 10 == 0:
                    print(f"📊 数据同步状态 (第{self.image_counters['camera0']}帧):")
                    print(f"  时间戳: {current_time:.3f}s")
                    print(f"  位置: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
                    print(f"  RGB文件: {image_filename}")
                    print(f"  深度文件: {depth_filename}")
                    print(f"  所有数据时间戳对齐: ✓")
                
                # 如果已到达目标点，进行额外记录
                if self.reached_target:
                    self.extra_frames_count += 1
                    print(f"✓ 额外记录第 {self.extra_frames_count} 帧")
                    
                    # 达到额外记录帧数后停止
                    if self.extra_frames_count >= self.max_extra_frames:
                        print(f"✓ 已完成额外记录 {self.max_extra_frames} 帧! 将停止记录...")
                        self.stop_recording(f"已到达目标点并完成额外记录{self.max_extra_frames}帧")
            
        except Exception as e:
            camera_names = {
                'camera0': '前',
                'camera1': '后', 
                'camera2': '左',
                'camera3': '右'
            }
            rospy.logerr(f"{camera_names.get(camera_id, camera_id)}摄像头同步处理错误: {str(e)}")
    
    def check_position_reached(self, position, orientation):
        """检查无人机是否到达目标点位置"""
        if self.target_position is None:
            return False
        
        # 计算位置差异
        pos_diff = np.sqrt(
            (position.x - self.target_position['position'].x) ** 2 +
            (position.y - self.target_position['position'].y) ** 2 +
            (position.z - self.target_position['position'].z) ** 2
        )
        
        # 每10帧输出一次调试信息
        self.debug_frame_count += 1
        if self.debug_frame_count % 10 == 0:
            target_pos = self.target_position['position']
            print(f"调试: 当前位置=({position.x:.3f}, {position.y:.3f}, {position.z:.3f})")
            print(f"     目标位置=({target_pos.x:.3f}, {target_pos.y:.3f}, {target_pos.z:.3f})")
            print(f"     位置差异={pos_diff:.3f}m, 容差={self.position_tolerance:.3f}m")
        
        # 计算姿态差异（四元数点积）
        target_ori = self.target_position['orientation']
        ori_dot = abs(
            orientation.x * target_ori.x +
            orientation.y * target_ori.y +
            orientation.z * target_ori.z +
            orientation.w * target_ori.w
        )
        # 四元数点积接近1表示方向一致
        ori_diff = 1.0 - ori_dot
        
        # 检查是否在容差范围内
        position_reached = pos_diff < self.position_tolerance
        orientation_reached = ori_diff < self.orientation_tolerance
        
        if position_reached:
            print(f"✓ 位置已到达！差异={pos_diff:.3f}m < 容差={self.position_tolerance:.3f}m")
        
        # 如果需要更严格的检查，可以同时要求位置和姿态都满足条件
        return position_reached # and orientation_reached
    
    def goal_callback(self, msg):
        # 记录触发信号到CSV文件（使用触发消息中的位置信息）
        current_time = time.time() - self.start_time if self.recording else 0.0
        with open(self.target_file, 'a') as f:
            pos = msg.pose.position
            ori = msg.pose.orientation
            f.write(f"{self.target_counter},{current_time:.3f},")
            f.write(f"{pos.x:.3f},{pos.y:.3f},{pos.z:.3f},")
            f.write(f"{ori.x:.6f},{ori.y:.6f},{ori.z:.6f},{ori.w:.6f}\n")
        
        self.target_counter += 1
        print(f"✓ 收到触发信号 #{self.target_counter}，已保存到 {os.path.basename(self.target_file)}")
        print(f"触发位置: X={pos.x:.2f}, Y={pos.y:.2f}, Z={pos.z:.2f}")
        
        if not self.recording:
            self.recording = True
            self.start_time = time.time()
            
            # 使用触发消息中的位置作为目标位置（而不是无人机当前位置）
            self.target_position = {
                'position': msg.pose.position,
                'orientation': msg.pose.orientation
            }
            
            rospy.loginfo("触发信号已接收! 开始记录位置数据和图像...")
            print("✓ 触发信号已接收! 开始记录位置数据和图像...")
            print(f"目标位置设定为: X={pos.x:.3f}, Y={pos.y:.3f}, Z={pos.z:.3f}")
    
    def stop_recording(self, reason="未知原因"):
        if self.recording:
            self.recording = False
            if self.file:
                self.file.close()
            rospy.loginfo(f"记录完成! ({reason}) 数据保存在 {self.output_dir}")
            print(f"✓ 记录完成! ({reason})")
            print(f"位置数据: {os.path.basename(self.data_file)}")
            print(f"目标点数据: {os.path.basename(self.target_file)}")
            print("图像统计:")
            print(f"  前摄像头: {self.image_counters['camera0']} 张 (保存在 front/)")
            print(f"  后摄像头: {self.image_counters['camera1']} 张 (保存在 back/)")  
            print(f"  左摄像头: {self.image_counters['camera2']} 张 (保存在 left/)")
            print(f"  右摄像头: {self.image_counters['camera3']} 张 (保存在 right/)")
            total_color_images = sum(self.image_counters.values())
            print(f"  彩色图像总计: {total_color_images} 张")
            
            print("深度图像统计:")
            print(f"  前深度摄像头: {self.depth_counters['camera0']} 张 (保存在 depth_images/front/)")
            print(f"  后深度摄像头: {self.depth_counters['camera1']} 张 (保存在 depth_images/back/)")  
            print(f"  左深度摄像头: {self.depth_counters['camera2']} 张 (保存在 depth_images/left/)")
            print(f"  右深度摄像头: {self.depth_counters['camera3']} 张 (保存在 depth_images/right/)")
            total_depth_images = sum(self.depth_counters.values())
            print(f"  深度图像总计: {total_depth_images} 张")
            print(f"  所有图像总计: {total_color_images + total_depth_images} 张")
            rospy.signal_shutdown(reason)

if __name__ == '__main__':
    if len(sys.argv) < 5:
        print("参数错误: 需要记录时长、输出目录、图像目录和数据文件")
        print("用法: python3 record_odom_image_trigger.py <duration> <output_dir> <image_dir> <data_file>")
        print("示例: python3 record_odom_image_trigger.py 30 /path/to/output /path/to/images data.csv")
        sys.exit(1)
        
    rospy.init_node('position_image_recorder_trigger', anonymous=True)
    recorder = PositionImageRecorder(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        if recorder.file:
            recorder.file.close()
        print("脚本被中断! 已保存数据.")