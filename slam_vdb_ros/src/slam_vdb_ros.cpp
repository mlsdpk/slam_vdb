#include "slam_vdb_ros/slam_vdb_ros.hpp"

namespace slam_vdb_ros {

SlamVDBROS::SlamVDBROS(const rclcpp::NodeOptions &options) : Node("slam_vdb", "", options)
{
  setUpROSParams();
  setUpROSInterfaces();
}

SlamVDBROS::~SlamVDBROS() {}

void SlamVDBROS::setUpROSParams()
{
  cloud_queue_size_ = static_cast<size_t>(this->declare_parameter("cloud_queue_size", 0));
  cloud_queue_size_ = std::max(cloud_queue_size_, 0ul);
  RCLCPP_INFO(get_logger(), "cloud_queue_size: %ld", cloud_queue_size_);
}

void SlamVDBROS::setUpROSInterfaces()
{
  // create a simple pointcloud subscriber
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_pointcloud", rclcpp::SensorDataQoS().keep_last(cloud_queue_size_),
      std::bind(&SlamVDBROS::pointcloudCallback, this, std::placeholders::_1));
}

void SlamVDBROS::pointcloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {}

}  // namespace slam_vdb_ros