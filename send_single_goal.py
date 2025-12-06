#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations
import sys

def send_single_goal(x, y, z, roll, pitch, yaw):
    """发送单个目标点"""
    # 初始化ROS节点
    rospy.init_node('send_single_goal', anonymous=True)
    
    # 创建发布器
    pub = rospy.Publisher('/uav_actions', PoseStamped, queue_size=1)
    
    # 等待发布器准备就绪
    rospy.sleep(1)
    
    # 创建目标点消息
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    
    # 设置位置
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.position.z = float(z)
    
    # 设置姿态 (从欧拉角转换为四元数)
    q = tf.transformations.quaternion_from_euler(float(roll), float(pitch), float(yaw))
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]
    
    # 发布目标点
    pub.publish(goal)
    
    rospy.loginfo("发送目标点: (%.3f, %.3f, %.3f) 姿态: (%.3f, %.3f, %.3f)", 
                  float(x), float(y), float(z), float(roll), float(pitch), float(yaw))
    print(f"✓ 成功发送目标点到 /uav_actions: ({float(x):.3f}, {float(y):.3f}, {float(z):.3f})")
    
    # 保持节点运行一小段时间确保消息发送
    rospy.sleep(0.5)

if __name__ == '__main__':
    if len(sys.argv) != 7:
        print("参数错误!")
        sys.exit(1)
    
    x, y, z, roll, pitch, yaw = sys.argv[1:7]
    
    try:
        send_single_goal(x, y, z, roll, pitch, yaw)
    except rospy.ROSInterruptException:
        pass
