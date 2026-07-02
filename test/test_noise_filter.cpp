#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "amr_sensor_fusion_demo/amr_noise_filter_node.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

// Helper to construct a mock PointCloud2 message for testing
sensor_msgs::msg::PointCloud2::SharedPtr create_mock_cloud(
  const std::vector<float>& x_coords,
  const std::vector<float>& y_coords,
  const std::vector<float>& z_coords)
{
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.frame_id = "laser_link";
  msg->height = 1;
  msg->width = x_coords.size();
  
  // Setup fields: x, y, z
  sensor_msgs::msg::PointField x_field;
  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x_field.count = 1;
  
  sensor_msgs::msg::PointField y_field;
  y_field.name = "y";
  y_field.offset = 4;
  y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  y_field.count = 1;
  
  sensor_msgs::msg::PointField z_field;
  z_field.name = "z";
  z_field.offset = 8;
  z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  z_field.count = 1;

  msg->fields = {x_field, y_field, z_field};
  msg->point_step = 12; // 3 floats * 4 bytes
  msg->row_step = msg->point_step * msg->width;
  msg->is_bigendian = false;
  msg->is_dense = true;
  
  msg->data.resize(msg->row_step);
  
  for (size_t i = 0; i < x_coords.size(); ++i) {
    size_t offset = i * msg->point_step;
    *reinterpret_cast<float*>(&msg->data[offset + 0]) = x_coords[i];
    *reinterpret_cast<float*>(&msg->data[offset + 4]) = y_coords[i];
    *reinterpret_cast<float*>(&msg->data[offset + 8]) = z_coords[i];
  }
  
  return msg;
}

TEST(PointCloudFilterTests, test_distance_and_height_filtering)
{
  rclcpp::NodeOptions options;
  // Instantiate the node under test
  auto node = std::make_shared<amr_sensor_fusion::PointCloudFilterNode>(options);
  
  // Create points:
  // P1: (1, 1, 0.5)   -> Range ~1.5m -> Keep
  // P2: (30, 0, 0)    -> Range 30m   -> Discard (exceeds distance_threshold of 25m)
  // P3: (2, 2, -0.5)  -> Range ~2.8m, Z below limit -> Discard (floor removal)
  // P4: (1, 1, 2.5)   -> Range ~2.8m, Z above limit -> Discard (ceiling removal)
  std::vector<float> x = {1.0f, 30.0f, 2.0f, 1.0f};
  std::vector<float> y = {1.0f, 0.0f, 2.0f, 1.0f};
  std::vector<float> z = {0.5f, 0.0f, -0.5f, 2.5f};
  
  auto input_cloud = create_mock_cloud(x, y, z);
  sensor_msgs::msg::PointCloud2 output_cloud;
  
  node->process_cloud(input_cloud, output_cloud);
  
  // Verify expected points: only 1 point should be retained
  EXPECT_EQ(output_cloud.width, 1u);
  EXPECT_EQ(output_cloud.height, 1u);
  
  // Verify coordinates of the retained point
  float rx = *reinterpret_cast<const float*>(&output_cloud.data[0]);
  float ry = *reinterpret_cast<const float*>(&output_cloud.data[4]);
  float rz = *reinterpret_cast<const float*>(&output_cloud.data[8]);
  
  EXPECT_NEAR(rx, 1.0f, 1e-5);
  EXPECT_NEAR(ry, 1.0f, 1e-5);
  EXPECT_NEAR(rz, 0.5f, 1e-5);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
