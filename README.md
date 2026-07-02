# Real-Time Point Cloud Filter for Industrial AMRs (`amr_sensor_fusion_demo`)

A focused ROS 2 C++ demo package for filtering raw 3D LiDAR-style point cloud streams. The package demonstrates range and height filtering for autonomous mobile robot (AMR) perception pipelines, with an emphasis on readable ROS 2 structure, direct `PointCloud2` parsing, and testable filter behavior.

---

## 🛠️ Key Technical Features

1.  **Direct Byte-Array Parsing:**
    Instead of converting ROS messages to memory-heavy objects such as PCL clouds at the subscriber level, this package reads the raw binary buffer of the `sensor_msgs/msg/PointCloud2` message using dynamic field-offset lookup. This keeps the demo focused on low-overhead filtering, but exact runtime should be benchmarked on the target robot/sensor before quoting performance numbers.
2.  **Concurrency-Safe Callback Groups:**
    The node maps callbacks to a `ReentrantCallbackGroup` and spins under a `MultiThreadedExecutor`. This prevents compute-heavy point cloud parsing loops from blocking critical, high-frequency odometry (`/odom`) or transform (`tf`) subscriptions.
3.  **ROS 2 Parameter Configuration:**
    Distance cutoff thresholds and height limits (`min_z_limit` / `max_z_limit` for floor reflections) are exposed as standard ROS 2 parameters for launch-time or startup configuration.
4.  **Google Test (gtest) Suite:**
    Includes a C++ unit test that verifies distance, ground-plane, and ceiling removal rules programmatically.

---

## 🏗️ Folder Structure

```text
amr_sensor_fusion_demo/
├── CMakeLists.txt              # Build system configuration
├── package.xml                 # ROS 2 package dependencies
├── README.md                   # Project documentation
├── AMR_ROBOTICS_DOMAIN_BRIEF.md
├── .gitignore                  # Git ignore rules
├── include/
│   └── amr_sensor_fusion_demo/
│       └── amr_noise_filter_node.hpp  # C++ Header file
├── src/
│   ├── amr_noise_filter_node.cpp      # Node implementation
│   └── main.cpp                       # Node entry point
└── test/
    └── test_noise_filter.cpp          # GTest unit tests
```

---

## 🚀 Building & Running

### Prerequisites
*   ROS 2 (Humble, Iron, or Jazzy)
*   colcon build tool

### Step 1: Clone and Build
Source your ROS 2 installation, then build the package inside your workspace:

```bash
# Navigate to your workspace
cd ~/amr_ws

# Build the package
colcon build --packages-select amr_sensor_fusion_demo

# Source the setup script
source install/setup.bash
```

### Step 2: Run the Filter Node
Run the node with default parameters:

```bash
ros2 run amr_sensor_fusion_demo amr_noise_filter_node
```

### Step 3: Run the GTest Unit Tests
To verify that all outlier filters function correctly, execute the test suite:

```bash
colcon test --packages-select amr_sensor_fusion_demo --event-handlers console_cohesion+
```

---

## 🗺️ How This Connects to AMR Robotics

This package maps to common perception challenges faced by industrial autonomous mobile robot (AMR) companies, especially teams working on intralogistics, warehouse automation, autonomous forklifts, tugger trains, and mobile robot fleets:

*   **Preventing False Stops:** Dust, steam, and floor reflections in a busy warehouse trigger false-positive costmap inflation, stopping vehicles unnecessarily. The floor and height limits filter out ground reflections while the range filter prevents distant noise from polluting the global costmap.
*   **High-Frequency Navigation Loops:** AMR local planners (Nav2) typically run at 10–20 Hz. This C++ node's direct byte-memory parsing is designed to keep filtering overhead low, leaving more CPU budget for path planning and localization.
*   **Testable Engineering Habit:** The included unit test shows how filter behavior can be checked automatically before the idea is expanded into a robot-ready component.

For domain context and project positioning, see [`AMR_ROBOTICS_DOMAIN_BRIEF.md`](AMR_ROBOTICS_DOMAIN_BRIEF.md).
