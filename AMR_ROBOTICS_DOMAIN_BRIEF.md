# AMR Robotics Domain Brief

Domain notes for discussing autonomous mobile robot (AMR) companies, especially in intralogistics, warehouse automation, production supply, autonomous forklifts, tugger trains, and mobile robot fleets.

## Domain Context

This demo is not meant to target one company. It is meant to show understanding of a class of robotics companies: teams that build or integrate autonomous mobile robots for industrial environments where perception, navigation, vehicle control, fleet behavior, and customer-site constraints all meet.

Typical companies in this domain work on:

- autonomous mobile robots for factories and warehouses
- autonomous forklifts and pallet-moving systems
- autonomous tugger trains and route-train automation
- mobile robot fleets for internal material flow
- ROS/ROS 2 software components, middleware, diagnostics, and deployment tooling
- customer-site integration where sensors, maps, networks, safety zones, and real operating conditions matter

The useful positioning angle is not "I like robots" in a generic way. It is:

"I am interested in the part where perception, ROS middleware, and real industrial constraints meet. AMRs in warehouses must keep moving safely even when the sensor data is messy: floor reflections, dust, pallets, people, and changing routes. That is exactly why LiDAR data quality and robust ROS software are interesting to me."

## Common Role Fit

This repo is relevant to roles that mention themes like:

- Develop modern ROS-based software components for mobile robot systems.
- Work in interdisciplinary teams on development and integration of autonomous mobile robots.
- Analyze and solve complex technical challenges.
- Continuously develop and optimize robot platforms.
- Expected skills include ROS/ROS2, component-based software development, middleware architectures, C++ or C#/.NET or Python, Linux, German and English.

## Demo Repository Content

This repository is a ROS 2 C++ demo package:

- `src/amr_noise_filter_node.cpp`: subscribes to `/input_points`, filters `sensor_msgs/msg/PointCloud2`, publishes `/filtered_points`.
- `include/amr_sensor_fusion_demo/amr_noise_filter_node.hpp`: node interface and filter parameters.
- `src/main.cpp`: runs the node with `MultiThreadedExecutor`.
- `test/test_noise_filter.cpp`: gtest for range, floor, and ceiling filtering.
- `CMakeLists.txt` and `package.xml`: ROS 2 ament package wiring.

The core technical story:

- Dynamic field-offset lookup for `x`, `y`, `z` inside `PointCloud2`.
- Direct byte-buffer parsing instead of converting the whole message to a PCL object.
- Range filter removes points beyond `distance_threshold`.
- Height filter removes floor/ceiling reflections using `min_z_limit` and `max_z_limit`.
- Reentrant callback group plus multi-threaded executor demonstrates awareness of callback isolation in ROS 2.
- Unit test proves the basic geometry filter behavior on a mock point cloud.

## Honest Boundaries

Use these boundaries. They make the story stronger because they are defensible.

- Do not say this demo is production safety software. It is a focused technical demo.
- Do not quote exact performance numbers from this repo until a benchmark is added.
- Do not claim the code dynamically updates parameters at runtime; it currently reads ROS 2 parameters at startup.
- Do not present C++ as professional project experience. Say: "My proven hands-on project experience is Python/ROS/Open3D for LiDAR evaluation. This C++ ROS 2 repo is a compact demo for understanding ROS 2 node structure, point-cloud message handling, and tradeoffs."
- Do not claim a specific company requested LiDAR filtering unless they confirm it. Say it is a relevant AMR perception inference from the domain and the ROS/software role.

## Short Project Pitch

English:

"My background is in electrical and microsystems engineering, with a strong focus on LiDAR validation, point-cloud processing, and reproducible technical evaluation. What interests me in industrial AMR companies is that the robots operate in real intralogistics environments, where sensor data quality, ROS middleware, and safe vehicle behavior all interact. My proven implementation experience is Python/ROS/Open3D. I prepared this small ROS 2 C++ package as a compact technical demo to show that I understand the software path: subscribing to `PointCloud2`, filtering noisy points, preserving message structure, publishing a clean output, and testing the filter behavior."

German:

"Mein Hintergrund liegt in Elektro- und Mikrosystemtechnik, mit Fokus auf LiDAR-Validierung, Punktwolkenverarbeitung und reproduzierbarer technischer Auswertung. An industrieller mobiler Robotik interessiert mich besonders, dass AMRs in echten Intralogistik-Umgebungen arbeiten. Dort treffen Sensordaten, ROS-Middleware und sicheres Fahrzeugverhalten direkt zusammen. Meine nachweisbare praktische Erfahrung liegt in Python, ROS und Open3D. Dieses kleine ROS-2-C++-Paket nutze ich als technisches Demo, um den Softwarepfad zu zeigen: `PointCloud2` lesen, verrauschte Punkte filtern, die Nachrichtenstruktur erhalten und das Filterverhalten testen."

## How To Walk Through The Repo

1. Start with the problem: AMRs depend on clean perception input.
2. Open `README.md` and explain the package at a high level.
3. Open `src/amr_noise_filter_node.cpp`:
   - field lookup for `x`, `y`, `z`
   - range check
   - `z` limits for floor/ceiling reflections
   - output message layout update
4. Open `src/main.cpp`:
   - explain why a `MultiThreadedExecutor` matters when sensor callbacks coexist with odometry, TF, diagnostics, or state updates.
5. Open `test/test_noise_filter.cpp`:
   - explain the test points and expected single retained point.
6. Close with next improvements:
   - add runtime parameter callback
   - add QoS profile for sensor data
   - add benchmark node or rosbag replay benchmark
   - add CI using a ROS 2 Humble container
   - add handling for endianness, datatype validation, NaNs, and malformed point steps

## Technical Discussion Points

Q: Why ROS 2 for AMRs?

A: ROS 2 gives modular nodes, pub/sub separation, parameters, launch files, lifecycle patterns, TF, and middleware choices. For AMRs, that helps separate sensors, localization, planning, vehicle control, diagnostics, and fleet interfaces.

Q: Why C++ here?

A: C++ is common in high-frequency robot software and many ROS 2 stacks. My proven experience is Python-based analysis and ROS/Open3D evaluation; this repo is a compact demo for understanding the C++ ROS 2 structure and discussing tradeoffs, not a claim of professional C++ project experience.

Q: What would you improve before deploying this?

A: I would add sensor QoS, runtime parameter updates, NaN/datatype/endianness guards, a rosbag replay benchmark, CI, and tests for malformed clouds. For a safety-relevant AMR, I would also treat this as only one layer before validated perception and safety logic.

Q: How do you use AI tools?

A: I use an LLM as a pair-programmer for boilerplate, tests, documentation, and alternative designs. I do not treat it as source of truth. For ROS, I verify with CLI tools, tests, logs, and real message behavior because QoS, TF timing, and sensor data issues must be proven in the system.

## Domain Questions To Discuss

- Which ROS versions and middleware settings are used in your current AMR stack?
- How is the team split between perception, navigation, vehicle control, fleet management, and commissioning?
- What are the most common field issues: sensor noise, localization, path planning, docking, customer environment changes, or fleet coordination?
- How do you test changes before they reach real vehicles: simulation, rosbag replay, HIL, test tracks, or customer pilots?
- For this role, what would successful contribution look like in the first three months?

## Closing Line

"I know I still have learning to do in production C++ robotics, but I am serious about the domain. I prepared this repo to make the technical discussion concrete: here is how I think about ROS 2, point-cloud data, testing, and honest tradeoffs."
