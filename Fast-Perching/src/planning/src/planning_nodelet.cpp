#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>
#include <traj_opt/poly_traj_utils.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <thread>
#include <vis_utils/vis_utils.hpp>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <traj_utils/PolyTraj.h>

namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");


enum StateMachine{
  INIT,
  IDLE,
  PLANNING,
  REPLAN,
  GOINGTOGOAL,
  EMERGENCY_STOP,
};

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  ros::Subscriber triger_sub_;
  ros::Subscriber odom_sub_;
  ros::Timer plan_timer_, target_listener_timer_;

  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;

  // NOTE planning or fake target
  bool target_ = false;

  // NOTE just for debug
  bool debug_ = false;
  bool once_ = false;
  bool debug_replan_ = false;

  double tracking_dur_, tracking_dist_, tolerance_d_;
  Eigen::Vector3d perching_p_, perching_v_, perching_axis_;
  double perching_theta_;

  Trajectory traj_;
  ros::Time replan_stamp_;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;

  bool replan = true;

  int plan_hz_ = 10;

  // for perching in unity
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  Eigen::Vector3d state_p_, state_v_, state_a_, state_j_;
  Eigen::Vector3d target_p, target_v;
  Eigen::Vector3d triger_position_;  // 存储从triger话题接收到的位置
  Eigen::Quaterniond target_q;
  Eigen::Quaterniond land_q;
  ros::Publisher traj_pub_, target_plane_pub_;

  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool target_received_ = ATOMIC_VAR_INIT(false);

  // for replan
  StateMachine state_machine_ = StateMachine::IDLE;
  double replan_time_;
  double max_replan_time_;
  double plan_start_time_;

  ros::Time Traj_start_time_;
  double Traj_total_time_;
  ros::Time loop_start_time_;
  double predicted_traj_start_time_;

  void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    // 从triger话题中提取位置信息
    triger_position_.x() = msgPtr->pose.position.x;
    triger_position_.y() = msgPtr->pose.position.y;
    triger_position_.z() = msgPtr->pose.position.z;
    
    triger_received_ = true;
  }

  void target_listener_Callback(const ros::TimerEvent& event) {
    if (!triger_received_ ) {
      return;
    }
    
    // 使用triger话题中的位置信息设置目标位置
    target_p = triger_position_;
    
    // 从轴角参数构建四元数姿态
    if (perching_axis_.norm() > 1e-6) {
        // 如果提供了旋转轴，使用轴角表示法
        Eigen::Vector3d axis = perching_axis_.normalized();
        target_q = Eigen::AngleAxisd(perching_theta_, axis);
    } else {
        // 如果没有提供旋转轴，假设绕Z轴旋转
        target_q = Eigen::AngleAxisd(perching_theta_, Eigen::Vector3d::UnitZ());
    }
        
    land_q = target_q;
    target_received_ = true;

    nav_msgs::Odometry target_plane;
    target_plane.header.stamp = ros::Time::now();
    target_plane.header.frame_id = "world";
    target_plane.pose.pose.position.x = target_p.x();
    target_plane.pose.pose.position.y = target_p.y();
    target_plane.pose.pose.position.z = target_p.z();
    target_plane.pose.pose.orientation.w = target_q.w();
    target_plane.pose.pose.orientation.x = target_q.x();
    target_plane.pose.pose.orientation.y = target_q.y();
    target_plane.pose.pose.orientation.z = target_q.z();
    
    // 发布目标平面
    target_plane_pub_.publish(target_plane);
    triger_received_ = false;
  }


  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    state_p_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    state_v_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    state_a_ << 0.0, 0.0, 0.0;  
    state_j_ << 0.0, 0.0, 0.0; 
  }

  void publish_traj(const Trajectory& traj, ros::Publisher& pub, 
                 int16_t drone_id, ros::Time Traj_start_time) {
      traj_utils::PolyTraj msg;
      std::vector<Piece> pieces = traj.getPieces();
      
      // 设置消息头
      msg.drone_id = drone_id;
      // msg.traj_id = traj_id;
      msg.start_time = Traj_start_time;  // 轨迹开始时间
      msg.order = 7;  
      
      // 预分配空间
      int piece_count = pieces.size();
      msg.duration.resize(piece_count);
      msg.coef_x.resize(8 * piece_count);
      msg.coef_y.resize(8 * piece_count);
      msg.coef_z.resize(8 * piece_count);
      
      // 填充数据
      for (int i = 0; i < piece_count; ++i) {
          const auto& piece = pieces[i];
          msg.duration[i] = piece.getDuration();
          
          Eigen::Matrix<double, 3, 8> coeff = piece.getCoeffMat();
          for (int j = 0; j < 8; ++j) {
              // 按段顺序存储系数
              int idx = i * 8 + j;
              msg.coef_x[idx] = coeff(0, j);  // x系数
              msg.coef_y[idx] = coeff(1, j);  // y系数
              msg.coef_z[idx] = coeff(2, j);  // z系数
          }
      }
      
      pub.publish(msg);
      // std::cout << "Published trajectory topic: " << pub.getTopic() << std::endl;
  }

  void debug_timer_callback(const ros::TimerEvent& event) {
    if (!target_received_) 
    {
      return;
    }

    Eigen::MatrixXd iniState;
    iniState.setZero(3, 4);

    if(state_machine_ == StateMachine::IDLE ||
      (  (state_machine_ == StateMachine::PLANNING || state_machine_ == StateMachine::REPLAN)
       &&(ros::Time::now() - loop_start_time_).toSec()>replan_time_                         ))
    {
      // target_received_ = false;
      loop_start_time_ = ros::Time::now();
      double current = loop_start_time_.toSec(); 
      // start new plan
      if(state_machine_ == StateMachine::IDLE)
      {
        state_machine_ = StateMachine::PLANNING;
        // initialize
        plan_start_time_ = -1;
        predicted_traj_start_time_ = -1;
        iniState.col(0) = state_p_;
        iniState.col(1) = state_v_;
        iniState.col(2) = state_a_;
        iniState.col(3) = state_j_;
      }
      else if(state_machine_ == StateMachine::PLANNING || 
              state_machine_ == StateMachine::REPLAN)
      {
        // 接近目标时，不需要再重规划，直接前进，return
        if((state_p_ - target_p).squaredNorm() < 1.0 ||
          traj_.getTotalDuration() < max_replan_time_)
        {
          state_machine_ = StateMachine::GOINGTOGOAL;
          return;
        }
        // 需要重规划了
        state_machine_ = StateMachine::REPLAN;
        predicted_traj_start_time_ = current + max_replan_time_ - plan_start_time_;
        
        iniState.col(0) = traj_.getPos(predicted_traj_start_time_);
        iniState.col(1) = traj_.getVel(predicted_traj_start_time_);
        iniState.col(2) = traj_.getAcc(predicted_traj_start_time_);
        iniState.col(3) = traj_.getJer(predicted_traj_start_time_);
        // 这里没有return，会接着执行
      }

      Trajectory traj;
      ROS_INFO("\033[0;30;43m start new plan \033[0m");
      bool generate_new_traj_success = false;
      std::cout << "iniState: \n"
                << iniState << std::endl;
      std::cout << "target_p: " << target_p.transpose() << std::endl;
      std::cout << "target_v: " << target_v.transpose() << std::endl;
      std::cout << "land_q: "
                << land_q.w() << ","
                << land_q.x() << ","
                << land_q.y() << ","
                << land_q.z() << "," << std::endl;
      generate_new_traj_success = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj);
      if (generate_new_traj_success) {
        visPtr_->visualize_traj(traj, "traj");
        Eigen::Vector3d tail_pos = traj.getPos(traj.getTotalDuration());
        Eigen::Vector3d tail_vel = traj.getVel(traj.getTotalDuration());
        visPtr_->visualize_arrow(tail_pos, tail_pos + 0.5 * tail_vel, "tail_vel");
        traj_ = traj;
      }
      // plan failed
      if (!generate_new_traj_success) {
        state_machine_ = REPLAN;
        ROS_ERROR("EMERGENCY_STOP!!!");
        return;
      }
      ROS_INFO("\033[0;30;46m all of plan time:%f \033[0m", (ros::Time::now().toSec()-current));
        
      // 发布轨迹
      if(plan_start_time_ < 0){
        Traj_start_time_ = ros::Time::now();
        plan_start_time_ = Traj_start_time_.toSec();
      }
      else{
        plan_start_time_ = current + max_replan_time_;
        Traj_start_time_ = ros::Time(plan_start_time_);
      }
      publish_traj(traj_, traj_pub_, 0, Traj_start_time_);
      Traj_total_time_ = traj_.getTotalDuration();
    }

    if((ros::Time::now() - Traj_start_time_).toSec() >= Traj_total_time_)
    {
      state_machine_ = StateMachine::IDLE;
      target_received_ = false;
      std::cout << "Traj finished, reset state_machine to IDLE" << std::endl;
    }
  }


  void init(ros::NodeHandle& nh) {
    // set parameters of planning
    nh.getParam("replan", debug_replan_);

    // NOTE once
    nh.getParam("perching_px", perching_p_.x());
    nh.getParam("perching_py", perching_p_.y());
    nh.getParam("perching_pz", perching_p_.z());
    nh.getParam("perching_vx", perching_v_.x());
    nh.getParam("perching_vy", perching_v_.y());
    nh.getParam("perching_vz", perching_v_.z());
    nh.getParam("perching_axis_x", perching_axis_.x());
    nh.getParam("perching_axis_y", perching_axis_.y());
    nh.getParam("perching_axis_z", perching_axis_.z());
    nh.getParam("perching_theta", perching_theta_);
    
    // 设置目标速度
    target_v = perching_v_;
    nh.param<double>("replan_time",replan_time_,100000000);
    nh.param<double>("max_replan_time", max_replan_time_, 0.08);

    visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);

    plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz_), &Nodelet::debug_timer_callback, this);
    target_listener_timer_ = nh.createTimer(ros::Duration(0.01), &Nodelet::target_listener_Callback, this);

    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("triger", 10, &Nodelet::triger_callback, this, ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/drone_0_visual_slam/odom", 10, &Nodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
    traj_pub_ = nh.advertise<traj_utils::PolyTraj>("trajectory_7_order", 10);
    target_plane_pub_ = nh.advertise<nav_msgs::Odometry>("/perching/target_plane", 10);
    ROS_WARN("Planning node initialized!");
  }

 public:

  Nodelet() : tf_listener_(tf_buffer_) {
        // 构造函数体可以为空，初始化工作在初始化列表中完成
  }

  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);