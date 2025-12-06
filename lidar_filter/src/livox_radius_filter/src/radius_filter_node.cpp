#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_ros/point_cloud.h>

class RadiusFilterNode
{
public:
    RadiusFilterNode()
    {
        ros::NodeHandle pnh("~");
        // 参数：半径，邻居最小数，输入/输出话题
        pnh.param("radius_search", radius_, 0.5);            // 默认 0.5 m
        pnh.param("min_neighbors", min_neighbors_, 5);      // 默认至少5个邻居
        pnh.param<std::string>("input_topic", input_topic_, "/livox/lidar");
        pnh.param<std::string>("output_topic", output_topic_, "/livox/lidar_filter");

        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        sub_ = nh_.subscribe(input_topic_, 2, &RadiusFilterNode::cloudCallback, this);

        ROS_INFO("RadiusFilterNode started. input: %s -> output: %s, radius: %.3f, min_neighbors: %d",
                 input_topic_.c_str(), output_topic_.c_str(), radius_, min_neighbors_);
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        // 转为 PCL 点云（使用 PointXYZI 尽量兼容强度字段）
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_out(new pcl::PointCloud<pcl::PointXYZI>());

        // 尝试把消息转换到 PointXYZI（若无 intensity 字段，强度设为 0）
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *pcl_in);

        // 如果点云类型不含 intensity（例如 PointXYZ），fromPCLPointCloud2 仍能填充 x,y,z，i 为 0
        // 应用 RadiusOutlierRemoval
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
        ror.setInputCloud(pcl_in);
        ror.setRadiusSearch(radius_);
        ror.setMinNeighborsInRadius(min_neighbors_);
        ror.filter(*pcl_out);

        // 转回 ROS 消息并发布
        sensor_msgs::PointCloud2 out_msg;
        pcl::PCLPointCloud2 pcl_out2;
        pcl::toPCLPointCloud2(*pcl_out, pcl_out2);
        pcl_conversions::fromPCL(pcl_out2, out_msg);
        out_msg.header = cloud_msg->header; // 保持原始时间戳和frame_id
        pub_.publish(out_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    double radius_;
    int min_neighbors_;
    std::string input_topic_, output_topic_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "livox_radius_filter");
    RadiusFilterNode node;
    ros::spin();
    return 0;
}
