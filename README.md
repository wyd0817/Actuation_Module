# Actuation Module for DART-LLM

The Actuation Module is a core component of the DART-LLM (Dependency-Aware Multi-Robot Task Decomposition and Execution using Large Language Models) system. It serves as the bridge between high-level commands and physical robot execution, primarily focusing on navigation control and coordination of multiple robots.

## Features

- **Navigation Functions**
  - Area avoidance control for all/specific robots
  - Target area guidance
  - Area access permission management
  - Return-to-start operations

- **ROS2 Integration**
  - Processes structured commands from Breakdown Function modules
  - Interfaces with ROS2 Navigation Stack
  - Manages real-time robot control and coordination

## Unity Simulation

1. **Launch ROS TCP Endpoint**:
   ```bash
   cd ~/share/ros2_ws
   source install/local_setup.bash
   ros2 launch ros_tcp_endpoint endpoint.py
   ```

2. **Launch OperaSim with Nav2**:
   ```bash
   cd ~/share/ros2_ws
   source install/local_setup.bash
   ros2 launch operasim_nav2 multi_robot_startup_launch.py
   ```

## Real Toy Robots

1. **Launch Zenoh Bridge ROS2dds**:
   ```bash
   cd ~/share/ros2_ws
   source install/setup.bash
   ros2 run zenoh_bridge_ros2dds zenoh_bridge_ros2dds -m peer
   ```

2. **Launch OperaSim with Nav2**:
   ```bash
   cd ~/share/ros2_ws
   source install/setup.bash
   ros2 launch operasim_nav2 multi_robot_startup_launch_4_toy_robots_launch.py
   ```

### Notes

- These commands assume you are working within the `~/share/ros2_ws` workspace directory. Adjust paths if your workspace differs.