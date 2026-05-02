# Research Track 2 - Assignment 1 (ROS 2 Version)

This repository contains a ROS 2 implementation for a robot navigation system using a **Client-Server architecture** based on Actions. The system allows a mobile robot to reach a specific target $(x, y, \theta)$ in a simulated environment using coordinate transformations.

---

## Technical Implementation

### 1. Action Mechanism (`rclcpp_action`)
The project utilizes ROS 2 Actions to manage navigation as a long-running, preemptible task.
*   **Purpose:** Actions are used instead of Services because navigation takes time. They allow the system to provide live feedback and support **cancellation** (preemption) if the user changes their mind.
*   **Client (`GoToClient`):** Handles user input via a dedicated thread. It sends the $(x, y, \theta)$ goal to the server and monitors the feedback. It allows the user to press 'c' to send an `async_cancel_goal` request.
*   **Server (`GoToServer`):** Receives the goal and executes the control loop in a separate thread to avoid blocking the ROS executor. It monitors the `is_canceling()` state to stop the robot safely if a cancellation is requested.

### 2. Frames and Coordinate Transformations (`tf2`)
The system relies heavily on the **TF2 library** to manage spatial relationships between the robot and its target.
*   **Frame Broadcasting:** The server broadcasts two main transforms:
    *   `world -> base_link`: Represents the robot's current position based on odometry.
    *   `world -> goal`: Represents the target position requested by the user.
*   **Frame Lookup:** Instead of calculating distances manually in the global map, the server uses `tf_buffer_->lookupTransform("base_link", "goal", ...)` to get the target's position **relative to the robot**.
*   **Control Logic:** By transforming the goal into the `base_link` frame, the robot simply calculates the distance and angle error (`atan2(dy, dx)`) relative to itself, making the movement logic robust and independent of the global starting point.

### 3. Component-Based Architecture
The project is implemented using **ROS 2 Components** (`rclcpp_components`):
*   **Modularization:** Both `GoToClient` and `GoToServer` are defined as classes inheriting from `rclcpp::Node` and registered via `RCLCPP_COMPONENTS_REGISTER_NODE`.
*   **Efficiency:** Components allow multiple nodes to be loaded into a single process (container), reducing communication overhead (intra-process communication) and memory usage.

---

## Launch File & Execution

The system uses a **Composable Node Container** to manage the lifecycle of the nodes.

### Launch Description
The `generate_launch_description()` in the Python launch file:
1.  Starts a `component_container`.
2.  Uses `xterm -e` as a prefix for the container, opening a dedicated terminal for user interaction.
3.  Dynamically loads the `GoToServer` and `GoToClient` plugins into the running container.

**To run the project:**
```bash
ros2 launch bme_gazebo_sensors container.launch.py
