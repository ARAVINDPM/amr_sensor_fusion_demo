cmake_minimum_required(VERSION 3.8)
project(amr_sensor_fusion_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Include directories
include_directories(include)

# Create shared library for the filter logic to enable clean unit testing
add_library(amr_filter_lib
  src/amr_noise_filter_node.cpp
)
ament_target_dependencies(amr_filter_lib
  rclcpp
  sensor_msgs
  pcl_conversions
)

# Create executable node
add_executable(amr_noise_filter_node src/main.cpp)
target_link_libraries(amr_noise_filter_node amr_filter_lib)
ament_target_dependencies(amr_noise_filter_node
  rclcpp
)

# Install rules
install(TARGETS
  amr_filter_lib
  amr_noise_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Testing configuration
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # Run standard linters (cppcheck, uncrustify, etc.)
  ament_lint_auto_find_test_dependencies()

  # Create gtest target
  ament_add_gtest(test_noise_filter test/test_noise_filter.cpp)
  target_link_libraries(test_noise_filter amr_filter_lib)
  ament_target_dependencies(test_noise_filter
    rclcpp
    sensor_msgs
  )
endif()

ament_package()
