 
#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <traj_opt/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <boost/make_shared.hpp>


ros::Publisher pos_cmd_pub;
quadrotor_msgs::PositionCommand cmd;

// 固定yaw角度参数
double fixed_yaw_ = 0.0;  // 弧度制，可以设置为任意固定值
bool use_velocity_yaw_ = false; // 是否使用基于速度的yaw计算
double yaw_offset_ = 0.0; // yaw偏移量
bool use_time_varying_yaw_ = false; // 是否使用时变yaw
double yaw_frequency_ = 1.0; // yaw变化频率(Hz)
double yaw_amplitude_ = 0.5; // yaw变化幅度(弧度)

// 从trigger话题获取的yaw角度
double trigger_yaw_ = 0.0;
bool use_trigger_yaw_ = true; // 是否使用从trigger话题获取的yaw
bool trigger_yaw_received_ = false; // 是否已经接收到trigger话题

bool receive_traj_ = false;
boost::shared_ptr<Trajectory> traj_;
boost::shared_ptr<Trajectory> new_traj_;
double traj_duration_ = 0.0;
double new_traj_duration_ = 0.0;
ros::Time start_time_;
ros::Time new_traj_start_time_;
int traj_id_;


// 处理trigger话题的回调函数
void triggerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // 从四元数提取yaw角度
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    
    // 转换为欧拉角
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // 更新trigger yaw
    trigger_yaw_ = yaw;
    trigger_yaw_received_ = true;
    
    ROS_INFO("Received trigger yaw: %.4f rad (%.2f deg)", yaw, yaw * 180.0 / M_PI);
}


void poly_7_TrajCallback(const traj_utils::PolyTraj::ConstPtr& msg)
{
     if (msg->order != 7) {
        ROS_ERROR("Invalid polynomial order: %d, expected 7", msg->order);
        return;
    }
    
    // 计算段数
    int piece_count = msg->duration.size();
    if (piece_count == 0) {
        ROS_WARN("Empty trajectory received");
        return;
    }
    
    // 验证系数数组长度
    if (msg->coef_x.size() != 8 * piece_count ||
        msg->coef_y.size() != 8 * piece_count ||
        msg->coef_z.size() != 8 * piece_count) 
    {
        ROS_ERROR("Coefficient array size mismatch");
        return;
    }
    
    // 准备重建数据
    std::vector<double> durs;
    std::vector<Eigen::Matrix<double, 3, 8>> cMats;
    durs.reserve(piece_count);
    cMats.reserve(piece_count);
    
    // 重建每段轨迹
    for (int i = 0; i < piece_count; ++i) 
    {
        durs.push_back(static_cast<double>(msg->duration[i]));
        Eigen::Matrix<double, 3, 8> coeffMat;
        for (int j = 0; j < 8; ++j) {
            int idx = i * 8 + j;
            coeffMat(0, j) = msg->coef_x[idx];  // x
            coeffMat(1, j) = msg->coef_y[idx];  // y
            coeffMat(2, j) = msg->coef_z[idx];  // z
        }
        cMats.push_back(coeffMat);
    }

    if(!traj_)
    {
        try 
        {
            traj_.reset(new Trajectory(durs, cMats)); 
            receive_traj_ = true;
            start_time_ = msg->start_time;
            traj_duration_ = 0;
            for (const auto& dur : durs) 
            {
                traj_duration_ += dur;
            }
        } 
        catch (const std::exception& e) 
        {
            ROS_ERROR("Failed to create trajectory: %s", e.what());
        }
    }
    else
    {
        try 
        {
            new_traj_.reset(new Trajectory(durs, cMats)); 
            receive_traj_ = true;
            new_traj_start_time_ = msg->start_time;
            new_traj_duration_ = 0;
            for (const auto& dur : durs) 
            {
                new_traj_duration_ += dur;
            }
        } 
        catch (const std::exception& e) 
        {
            ROS_ERROR("Failed to create trajectory: %s", e.what());
        }
    }
    
}


void cmdCallback(const ros::TimerEvent& e)
{
    if (!receive_traj_ || !traj_) {
        ROS_WARN_THROTTLE(1.0, "No trajectory received yet");
        return;
    }
    
    // 计算当前时间点
    double t = (ros::Time::now() - start_time_).toSec();
    ros::Time current_time = ros::Time::now();

    if (new_traj_ && current_time >= new_traj_start_time_) {
        traj_ = new_traj_;
        start_time_ = new_traj_start_time_;
        traj_duration_ = new_traj_duration_;
        new_traj_.reset();
        ROS_INFO("Switched to new trajectory.");
    }

    if(traj_)
    {
        if (t > traj_duration_) {
            ROS_WARN_THROTTLE(1.0, "Trajectory finished");
            receive_traj_ = false;
            return;
        }
        // 评估轨迹
        Eigen::Vector3d pos, vel, acc , jer;
        pos = traj_->getPos(t);
        vel = traj_->getVel(t);
        acc = traj_->getAcc(t);
        jer = traj_->getJer(t);
        
        // 填充位置命令消息
        cmd.header.stamp = ros::Time::now();
        cmd.position.x = pos.x();
        cmd.position.y = pos.y();
        cmd.position.z = pos.z();
        cmd.velocity.x = vel.x();
        cmd.velocity.y = vel.y();
        cmd.velocity.z = vel.z();
        cmd.acceleration.x = acc.x();
        cmd.acceleration.y = acc.y();
        cmd.acceleration.z = acc.z();
        // 计算yaw角度
        double yaw_cmd = fixed_yaw_;
        double yaw_dot_cmd = 0.0;
        
        if (use_trigger_yaw_ && trigger_yaw_received_) {
            // 使用从trigger话题获取的yaw角度
            yaw_cmd = trigger_yaw_;
        } else if (use_time_varying_yaw_) {
            // 时变yaw: yaw = fixed_yaw + amplitude * sin(2*pi*freq*t)
            double time_varying_yaw = yaw_amplitude_ * sin(2.0 * M_PI * yaw_frequency_ * t);
            yaw_cmd = fixed_yaw_ + time_varying_yaw;
            yaw_dot_cmd = yaw_amplitude_ * 2.0 * M_PI * yaw_frequency_ * cos(2.0 * M_PI * yaw_frequency_ * t);
        } else if (use_velocity_yaw_ && vel.norm() > 0.1) {
            // 基于速度方向计算yaw角度
            yaw_cmd = atan2(vel.y(), vel.x()) + yaw_offset_;
            yaw_dot_cmd = 0.0; // 简化处理，不计算yaw_dot
        }
        
        cmd.jerk.x = jer(0);
        cmd.jerk.y = jer(1);
        cmd.jerk.z = jer(2);
        cmd.yaw = yaw_cmd; 
        cmd.yaw_dot = yaw_dot_cmd;
        
        // 发布位置命令
        // std::cout<< "pos"<< pos.x()<<" "<<pos.y()<<" "<<pos.z()<<std::endl;
        // std::string topic = pos_cmd_pub.getTopic();
        // ROS_INFO("Advertise topioc : %s", topic.c_str());
        pos_cmd_pub.publish(cmd);
    }
}


void Test_move_Callback(const ros::TimerEvent& e)
{
    static ros::Time start_time = ros::Time::now();
    double t = (ros::Time::now() - start_time).toSec();

    const double max_value = 1;  
    const double freq = 1;  // Hz

    double value = max_value * std::sin(2 * M_PI * freq * t);
    double value_dot = max_value * 2 * M_PI * freq * std::cos(2 * M_PI * freq * t);

    // 更新 yaw，保持其他指令不变
    cmd.header.stamp = ros::Time::now();
    cmd.position.x = 4;
    cmd.position.y = 1;
    cmd.position.z = 2;
    cmd.velocity.x = 0;
    cmd.velocity.y = 0;
    cmd.velocity.z = 0;
    cmd.acceleration.x = -3;
    cmd.acceleration.y = -3;
    cmd.acceleration.z = 0;
    cmd.jerk.x = 0;
    cmd.jerk.y = 0;
    cmd.jerk.z = 0;
    cmd.yaw = 0;
    cmd.yaw_dot = 0;

    pos_cmd_pub.publish(cmd);
}


 int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server_7_order");
  ros::NodeHandle nh("~");
  
  // 读取固定yaw参数
  nh.param<double>("fixed_yaw", fixed_yaw_, 1.57); // 默认90度（弧度制）
  nh.param<bool>("use_velocity_yaw", use_velocity_yaw_, false); // 是否使用速度方向yaw
  nh.param<double>("yaw_offset", yaw_offset_, 0.0); // yaw偏移量
  nh.param<bool>("use_time_varying_yaw", use_time_varying_yaw_, false); // 是否使用时变yaw
  nh.param<double>("yaw_frequency", yaw_frequency_, 1.0); // yaw变化频率
  nh.param<double>("yaw_amplitude", yaw_amplitude_, 0.5); // yaw变化幅度
  nh.param<bool>("use_trigger_yaw", use_trigger_yaw_, true); // 是否使用trigger话题的yaw
  
  ROS_INFO("Yaw configuration: fixed_yaw=%.2f, use_velocity_yaw=%d, yaw_offset=%.2f, use_time_varying_yaw=%d, freq=%.2f, amp=%.2f, use_trigger_yaw=%d", 
           fixed_yaw_, use_velocity_yaw_, yaw_offset_, use_time_varying_yaw_, yaw_frequency_, yaw_amplitude_, use_trigger_yaw_);

  ros::Subscriber poly_traj_sub = nh.subscribe("/drone0/planning/trajectory_7_order", 10, poly_7_TrajCallback);
  ros::Subscriber trigger_sub = nh.subscribe("/triger", 10, triggerCallback);
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/drone_0_planning/pos_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);
//   ros::Timer yaw_update_timer = nh.createTimer(ros::Duration(0.01), Test_move_Callback);

  ros::Duration(1.0).sleep();

   ROS_WARN("Trajectory server for 7th order polynomial initialized.");

  ros::spin();

  return 0;
}