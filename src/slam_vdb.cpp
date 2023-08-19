#include "slam_vdb/slam_vdb.hpp"

namespace slam_vdb {

SlamVDB::SlamVDB(const rclcpp::NodeOptions &options)
    : Node("slam_vdb", "", options) {
  setUpROSParams();
  setUpROSInterfaces();
}

SlamVDB::~SlamVDB() {}

void SlamVDB::setUpROSParams() {
  cloud_queue_size_ =
      static_cast<size_t>(this->declare_parameter("cloud_queue_size", 0));
  cloud_queue_size_ = std::max(cloud_queue_size_, 0ul);
  RCLCPP_INFO(get_logger(), "cloud_queue_size: %ld", cloud_queue_size_);
}

void SlamVDB::setUpROSInterfaces() {
  // create a simple pointcloud subscriber
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_pointcloud", rclcpp::SensorDataQoS().keep_last(cloud_queue_size_),
      std::bind(&SlamVDB::pointcloudCallback, this, std::placeholders::_1));
}

void SlamVDB::pointcloudCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {}

} // namespace slam_vdb