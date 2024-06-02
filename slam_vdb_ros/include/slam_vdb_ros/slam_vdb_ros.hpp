#ifndef SLAM_VDB__SLAM_VDB_HPP_
#define SLAM_VDB__SLAM_VDB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace slam_vdb_ros {

class SlamVDBROS : public rclcpp::Node
{
public:
  explicit SlamVDBROS(const rclcpp::NodeOptions &options);
  virtual ~SlamVDBROS();

private:
  void setUpROSParams();
  void setUpROSInterfaces();

  void pointcloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

private:
  size_t cloud_queue_size_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
};

}  // namespace slam_vdb_ros

#endif  // SLAM_VDB__SLAM_VDB_HPP_