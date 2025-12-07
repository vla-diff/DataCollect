#!/usr/bin/env python3
import rospy
import math
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry
import tf.transformations as tf_trans
from geometry_msgs.msg import TransformStamped

class DroneTFPublisher:
    def __init__(self):
        rospy.init_node('drone_tf_publisher')
        
        # 创建静态 TF 广播器
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # 发布相机到机体的静态变换
        self.publish_camera_static_transform()
        
        # 创建动态 TF 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 订阅视觉 SLAM 的里程计话题
        self.odom_sub = rospy.Subscriber(
            '/drone_0_visual_slam/odom', 
            Odometry, 
            self.odom_callback
        )
        
        rospy.loginfo("Drone TF Publisher started.")
        
    def publish_camera_static_transform(self):
        # 相机到机体的固定变换
        static_transform = TransformStamped()
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = 'drone_body'
        static_transform.child_frame_id = 'camera_color_frame'
        
        # 设置平移 (X:0.15, Y:0.1, Z:0)
        static_transform.transform.translation.x = 0.2
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.1
        
        angle_deg = 15.0
        angle_rad = math.radians(angle_deg)
        cos_val = math.cos(angle_rad)
        sin_val = math.sin(angle_rad)
        print("Cosine: ", cos_val)
        print("Sine: ", sin_val)
        # 构建旋转矩阵 (3x3)
        # 行0: X_new = [0, -1, 0]
        # 行1: Y_new = Z × X = [sin15, 0, -cos15]
        # 行2: Z_new = [cos15, 0, sin15]
        rot_matrix_3x3 = (
            (0,      sin_val   , cos_val),      # X 轴
            (-1,            0,      0), # Y 轴 (叉积 Z × X)
            (0,      -cos_val ,  sin_val)   # Z 轴
        ) 
        # 对3x3旋转矩阵进行转置
        rot_matrix_4x4 = (
            (rot_matrix_3x3[0][0], rot_matrix_3x3[0][1], rot_matrix_3x3[0][2], 0),
            (rot_matrix_3x3[1][0], rot_matrix_3x3[1][1], rot_matrix_3x3[1][2], 0),
            (rot_matrix_3x3[2][0], rot_matrix_3x3[2][1], rot_matrix_3x3[2][2], 0),
            (0,                  0,                  0,                  1)
        )
        
        # 将4x4齐次变换矩阵转换为四元数
        quat = tf_conversions.transformations.quaternion_from_matrix(rot_matrix_4x4)

        print("Quaternion: ", quat)
        
        # 设置四元数
        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]
            
        # 发布静态变换（只需发布一次）
        self.static_broadcaster.sendTransform(static_transform)
        
    def odom_callback(self, msg):
        # 发布机体到世界的动态变换
        body_to_world = TransformStamped()
        body_to_world.header.stamp = rospy.Time.now()
        body_to_world.header.frame_id = 'world'
        body_to_world.child_frame_id = 'drone_body'
        
        body_to_world.transform.translation = msg.pose.pose.position
        body_to_world.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(body_to_world)

if __name__ == '__main__':
    try:
        node = DroneTFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
