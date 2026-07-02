#include "amr_sensor_fusion_demo/amr_noise_filter_node.hpp"
#include <vector>
#include <cmath>

namespace amr_sensor_fusion
{

PointCloudFilterNode::PointCloudFilterNode(const rclcpp::NodeOptions & options)
: Node("amr_noise_filter_node", options)
{
  // Declare parameters
  this->declare_parameter<double>("distance_threshold", 25.0); // max range for indoor AMR
  this->declare_parameter<double>("min_z_limit", -0.1);         // ground plane buffer
  this->declare_parameter<double>("max_z_limit", 2.0);          // vertical height cutoff
  this->declare_parameter<bool>("use_floor_removal", true);

  // Retrieve parameters
  this->get_parameter("distance_threshold", distance_threshold_);
  this->get_parameter("min_z_limit", min_z_limit_);
  this->get_parameter("max_z_limit", max_z_limit_);
  this->get_parameter("use_floor_removal", use_floor_removal_);

  // Create publisher and subscriber (using reentrant callback group for multi-threading robustness)
  auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = callback_group;

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/input_points", 10,
    std::bind(&PointCloudFilterNode::cloud_callback, this, std::placeholders::_1),
    sub_options);

  RCLCPP_INFO(this->get_logger(), "AMR LiDAR Noise Filter initialized.");
}

void PointCloudFilterNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  auto filtered_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  process_cloud(msg, *filtered_msg);
  publisher_->publish(std::move(filtered_msg));
}

void PointCloudFilterNode::process_cloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  sensor_msgs::msg::PointCloud2 & filtered_msg) const
{
  // 1. Locate fields dynamically in the serialized binary buffer
  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;

  for (const auto & field : msg->fields) {
    if (field.name == "x") x_offset = field.offset;
    else if (field.name == "y") y_offset = field.offset;
    else if (field.name == "z") z_offset = field.offset;
  }

  if (x_offset == -1 || y_offset == -1 || z_offset == -1) {
    RCLCPP_WARN(this->get_logger(), "Point cloud message misses standard x, y, or z coordinate fields.");
    return;
  }

  // Set message headers
  filtered_msg.header = msg->header;
  filtered_msg.fields = msg->fields;
  filtered_msg.point_step = msg->point_step;
  filtered_msg.is_bigendian = msg->is_bigendian;
  filtered_msg.is_dense = true;

  std::vector<uint8_t> clean_data;
  clean_data.reserve(msg->data.size());

  const int num_points = msg->width * msg->height;
  const int point_step = msg->point_step;
  int retained_points = 0;

  // Real-time zero-copy byte-array parser
  for (int i = 0; i < num_points; ++i) {
    const size_t byte_index = i * point_step;

    // Direct memory cast (zero copy parsing of the payload)
    float x = *reinterpret_cast<const float *>(&msg->data[byte_index + x_offset]);
    float y = *reinterpret_cast<const float *>(&msg->data[byte_index + y_offset]);
    float z = *reinterpret_cast<const float *>(&msg->data[byte_index + z_offset]);

    // Apply geometry filtering criteria
    double range = std::sqrt(x * x + y * y + z * z);
    if (range > distance_threshold_) {
      continue; // out of range
    }

    if (use_floor_removal_) {
      if (z < min_z_limit_ || z > max_z_limit_) {
        continue; // floor reflection / ceiling crop
      }
    }

    // Retain point (direct copy of point byte sequence)
    const uint8_t * point_ptr = &msg->data[byte_index];
    clean_data.insert(clean_data.end(), point_ptr, point_ptr + point_step);
    retained_points++;
  }

  // Update layout info
  filtered_msg.data = std::move(clean_data);
  filtered_msg.width = retained_points;
  filtered_msg.height = 1;
  filtered_msg.row_step = retained_points * point_step;
}

}  // namespace amr_sensor_fusion
