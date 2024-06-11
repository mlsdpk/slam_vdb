#include <memory>

#include "slam_vdb_ros/slam_vdb_ros.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<slam_vdb_ros::SlamVDBROS>(options);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}