#ifndef SLAM_VDB__SLAM_VDB_HPP_
#define SLAM_VDB__SLAM_VDB_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

// nanoVDB
#include "nanovdb/NanoVDB.h"

namespace slam_vdb {

class SlamVDB : public rclcpp::Node {
public:
  explicit SlamVDB(const rclcpp::NodeOptions &options);
  virtual ~SlamVDB();

private:
  void setUpROSParams();
  void setUpROSInterfaces();

  void pointcloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

private:
  size_t cloud_queue_size_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
};

} // namespace slam_vdb

#endif // SLAM_VDB__SLAM_VDB_HPP_