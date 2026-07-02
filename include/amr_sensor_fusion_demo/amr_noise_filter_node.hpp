#ifndef AMR_SENSOR_FUSION_DEMO__AMR_NOISE_FILTER_NODE_HPP_
#define AMR_SENSOR_FUSION_DEMO__AMR_NOISE_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>
#include <string>

namespace amr_sensor_fusion
{

class PointCloudFilterNode : public rclcpp::Node
{
public:
  explicit PointCloudFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~PointCloudFilterNode() = default;

  // Process and filter the raw byte stream of a ROS PointCloud2 message
  void process_cloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    sensor_msgs::msg::PointCloud2 & filtered_msg) const;

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  // Parameters
  double distance_threshold_;
  double min_z_limit_;
  double max_z_limit_;
  bool use_floor_removal_;
};

}  // namespace amr_sensor_fusion

#endif  // AMR_SENSOR_FUSION_DEMO__AMR_NOISE_FILTER_NODE_HPP_
