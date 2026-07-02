#include <rclcpp/rclcpp.hpp>
#include "amr_sensor_fusion_demo/amr_noise_filter_node.hpp"
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Use a MultiThreadedExecutor to showcase advanced concurrent node architecture
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<amr_sensor_fusion::PointCloudFilterNode>();
  
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
