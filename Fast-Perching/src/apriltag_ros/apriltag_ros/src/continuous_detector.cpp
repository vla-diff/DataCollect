/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
void ContinuousDetector::onInit ()
{
  ROS_INFO("\033[34m Initializing ContinuousDetector\033[0m");
  NODELET_INFO("Nodelet is alive!");
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& pnh = getPrivateNodeHandle();

  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));

  std::string transport_hint;
  pnh.param<std::string>("transport_hint", transport_hint, "compressed");

  int queue_size;
  pnh.param<int>("queue_size", queue_size, 1);
  // camera_image_subscriber_ =
  //     it_->subscribeCamera("image_rect", queue_size,
  //                         &ContinuousDetector::imageCallback, this,
  //                         image_transport::TransportHints(transport_hint));
  // std::cout << "实际订阅: " << camera_image_subscriber_.getTopic() << std::endl;
  // // 在onInit()函数中添加
  // ROS_INFO("camera_info: %s", camera_image_subscriber_.getInfoTopic().c_str());

  std::string image_topic,image_info_topic,odom_topic;
  pnh.param<std::string>("image_topic", image_topic, "/camera/color/image/compressed");
  pnh.param<std::string>("image_info_topic", image_info_topic, "/camera/color/info");  
  pnh.param<std::string>("odom_topic", odom_topic, "/groundTruth/poseStamped");



  // 初始化消息过滤器订阅器
  image_sub_.subscribe(nh, image_topic, queue_size);
  info_sub_.subscribe(nh, image_info_topic, queue_size); // 确保话题名匹配
  odom_sub_.subscribe(nh, odom_topic, queue_size*5);        // /odom 话题
  
  sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
      SyncPolicy(static_cast<uint32_t>(queue_size * 5)),  // 使用上面配置好的策略
      image_sub_, info_sub_, odom_sub_));

  sync_->registerCallback(boost::bind(
      &ContinuousDetector::syncedImageCallback, this, _1, _2, _3));

  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

  refresh_params_service_ =
      pnh.advertiseService("refresh_tag_params", 
                          &ContinuousDetector::refreshParamsCallback, this);
}

void ContinuousDetector::refreshTagParameters()
{
  // Resetting the tag detector will cause a new param server lookup
  // So if the parameters have changed (by someone/something), 
  // they will be updated dynamically
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  ros::NodeHandle& pnh = getPrivateNodeHandle();
  tag_detector_.reset(new TagDetector(pnh));
}

bool ContinuousDetector::refreshParamsCallback(std_srvs::Empty::Request& req,
                                               std_srvs::Empty::Response& res)
{
  refreshTagParameters();
  return true;
}

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  // std::cout<<"\033[34m Recieved Image\033[0m"<<std::endl;
  std::scoped_lock<std::mutex> lock(detection_mutex_);
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_.publish(
      tag_detector_->detectTags(cv_image_,camera_info));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

void ContinuousDetector::syncedImageCallback(
  const sensor_msgs::CompressedImageConstPtr& image_rect,  // 压缩图像
  const sensor_msgs::CameraInfoConstPtr& camera_info,
  const geometry_msgs::PoseStampedConstPtr& odom)
{

  // 获取时间戳
  // ros::Time t_img = image_rect->header.stamp;
  // ros::Time t_info = camera_info->header.stamp;
  // ros::Time t_odom = odom->header.stamp;

  // // 打印时间戳（秒+纳秒）
  // ROS_INFO_STREAM("Timestamps:"
  //   << "\n - Image:      " << t_img.toSec()
  //   << "\n - CameraInfo: " << t_info.toSec()
  //   << "\n - Odometry:   " << t_odom.toSec());

  // // 计算两两之间的差值（单位：秒）
  // double dt_img_info = fabs((t_img - t_info).toSec());
  // double dt_img_odom = fabs((t_img - t_odom).toSec());
  // double dt_info_odom = fabs((t_info - t_odom).toSec());

  // ROS_INFO_STREAM("Time differences (sec):"
  //   << "\n - Image vs CameraInfo: " << dt_img_info
  //   << "\n - Image vs Odometry:   " << dt_img_odom
  //   << "\n - CameraInfo vs Odom:  " << dt_info_odom);




  // std::cout << "\033[34m Recieved Synced Image\033[0m" << std::endl;
    std::scoped_lock<std::mutex> lock(detection_mutex_);

    // Lazy updates:
    // When there are no subscribers _and_ when tf is not published,
    // skip detection.
    if (tag_detections_publisher_.getNumSubscribers() == 0 &&
        tag_detections_image_publisher_.getNumSubscribers() == 0 &&
        !tag_detector_->get_publish_tf())
    {
      // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
      return;
    }

    // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
    // AprilTag 2 on the iamge
    try {
      // 1. 用 OpenCV 解压压缩图像数据（CompressedImage 的 data 字段是压缩字节流）
      cv::Mat image_mat = cv::imdecode(cv::Mat(image_rect->data), cv::IMREAD_COLOR);
      if (image_mat.empty()) {
        ROS_ERROR("decompression failed: image data is empty");
        return;
      }

      // 2. 手动构造 CvImagePtr，包含图像数据、编码格式和头部信息
      cv_image_.reset(new cv_bridge::CvImage());
      cv_image_->header = image_rect->header;  // 复制时间戳、坐标系等头部信息
      cv_image_->encoding = "bgr8";           // 解压后默认是 BGR 格式（OpenCV 标准）
      cv_image_->image = image_mat;           // 赋值解压后的 cv::Mat
    } 
    catch (cv::Exception& e) {
      ROS_ERROR("OpenCV decompression error: %s", e.what());
      return;
    }
    catch (std::exception& e) {
      ROS_ERROR("other error: %s", e.what());
      return;
    }


    // Publish detected tags in the image by AprilTag 2
    tag_detections_publisher_.publish(
        tag_detector_->detectTags_odom(cv_image_,camera_info,odom));

    // Publish the camera image overlaid by outlines of the detected tags and
    // their payload values
    if (draw_tag_detections_image_)
    {
      tag_detector_->drawDetections(cv_image_);
      tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
    }


}


} // namespace apriltag_ros
