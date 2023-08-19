#include "slam_vdb/slam_vdb.hpp"

#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<slam_vdb::SlamVDB>(options);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}