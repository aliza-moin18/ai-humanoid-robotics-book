---
sidebar_position: 3
---

# Section 1.3: Integration between Gazebo and Unity

## Overview

This section explains how to establish communication between Gazebo and Unity for digital twin simulation. The integration allows for synchronized physics simulation in Gazebo with high-fidelity visualization in Unity.

## Integration Architecture

The Gazebo-Unity integration typically uses the following architecture:

- **Gazebo** handles physics simulation and sensor data generation
- **ROS 2** acts as the communication middleware
- **Bridge nodes** translate between Gazebo and Unity messages
- **Unity** provides visualization and user interaction

## Required Components

### ROS 2 Installation

1. Install ROS 2 Humble Hawksbill (recommended for robotics applications):
   - Follow the official installation guide: https://docs.ros.org/en/humble/Installation.html

2. Verify ROS 2 installation:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ```

### Unity ROS TCP Connector

1. Download the Unity ROS TCP Connector package from the Unity Asset Store or GitHub
2. Import the package into your Unity project
3. Configure the connector to connect to your ROS 2 network

### Gazebo ROS Packages

1. Install gazebo_ros_pkgs:
   ```bash
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```

2. Verify installation:
   ```bash
   ros2 pkg list | grep gazebo
   ```

## Implementation Steps

### Step 1: Network Configuration

1. Set up ROS 2 environment variables:
   ```bash
   export ROS_DOMAIN_ID=0
   export ROS_LOCALHOST_ONLY=0  # For multi-machine setup
   ```

2. Configure network settings to allow communication between Gazebo and Unity processes

### Step 2: Message Bridge Setup

1. Create a bridge node that subscribes to Gazebo topics and publishes to Unity:
   ```cpp
   // Example bridge node structure
   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"
   
   class GazeboUnityBridge : public rclcpp::Node
   {
   public:
     GazeboUnityBridge() : Node("gazebo_unity_bridge")
     {
       // Initialize publishers and subscribers
     }
   };
   ```

2. The bridge should handle:
   - Robot state messages (joint positions, poses)
   - Sensor data (LiDAR, IMU, cameras)
   - Control commands from Unity

### Step 3: Synchronization Protocol

1. Implement time synchronization between Gazebo and Unity
2. Establish update rate matching (typically 50-100Hz for real-time simulation)
3. Handle latency compensation for smooth visualization

### Step 4: State Mapping

1. Map Gazebo model states to Unity GameObjects:
   - Robot joint positions → Unity joint rotations
   - Sensor positions → Unity sensor placements
   - Physics properties → Unity physics materials

2. Implement transforms for coordinate system alignment:
   - Gazebo uses ENU (East-North-Up)
   - Unity uses Left-Handed Y-Up (X-right, Y-up, Z-forward)

## Verification

Test the integration with a simple robot model:

1. Launch Gazebo with a test world:
   ```bash
   gz sim -v 4 empty.sdf
   ```

2. Launch your bridge node

3. Start Unity scene with the robot model

4. Verify that:
   - Robot movements in Gazebo are reflected in Unity
   - Sensor data from Gazebo appears in Unity
   - Control inputs from Unity affect Gazebo simulation

## Troubleshooting

- If communication fails, check ROS_DOMAIN_ID settings on both systems
- For synchronization issues, verify time settings and update rates
- For coordinate system mismatches, check transform implementations

## Next Steps

After successfully establishing the Gazebo-Unity integration, proceed to [Section 1.4: Verification of Installation](../chapter-1/section-1-4-verification) to complete the setup process.