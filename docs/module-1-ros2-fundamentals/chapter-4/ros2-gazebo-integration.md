---
sidebar_position: 2
---

# ROS 2 and Gazebo Integration

## Overview

Gazebo is a 3D simulation environment that is commonly used with ROS 2 for robot simulation and testing. This section covers how to integrate ROS 2 with Gazebo to create realistic simulation environments for your robots.

## Gazebo Installation

First, ensure you have Gazebo installed with ROS 2 support:

```bash
sudo apt install ros-humble-gazebo-*
```

## Launching Gazebo with ROS 2

To launch Gazebo with ROS 2 support, you can use the following command:

```bash
# Launch Gazebo with GUI
ros2 launch gazebo_ros gazebo.launch.py

# Launch Gazebo without GUI (headless)
ros2 launch gazebo_ros gazebo.launch.py gui:=false
```

## Spawning Robots in Gazebo

Once Gazebo is running, you can spawn robots using the `spawn_entity` service:

```bash
# Spawn a robot from a URDF file
ros2 run gazebo_ros spawn_entity.py -file /path/to/robot.urdf -entity my_robot -x 0 -y 0 -z 1
```

## Gazebo Plugins for ROS 2

Gazebo uses plugins to interface with ROS 2. Common plugins include:

- `libgazebo_ros_diff_drive.so`: Differential drive controller
- `libgazebo_ros_joint_state_publisher.so`: Joint state publisher
- `libgazebo_ros_imu.so`: IMU sensor plugin
- `libgazebo_ros_laser.so`: Laser sensor plugin

## Example: Creating a Simple Robot Model for Gazebo

Here's a URDF with Gazebo-specific extensions:

```xml
<?xml version="1.0"?>
<robot name="gazebo_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 0" rpy="1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Gazebo-specific extensions -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
  </gazebo>

  <!-- Differential drive plugin -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_wheel_tf>true</publish_wheel_tf>
      <publish_odom_tf>true</publish_odom_tf>
    </plugin>
  </gazebo>
</robot>
```

## Launch Files for Simulation

You can create launch files to start both Gazebo and your robot controllers simultaneously:

```python
# launch/robot_world.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution(
        [FindPackageShare("my_robot_description"), "worlds", "simple_room.world"]
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]),
        launch_arguments={"world": world}.items(),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_robot"],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])
```

## Best Practices

- Test your robot controllers in simulation before deploying to hardware
- Use realistic physics parameters in your URDF
- Validate sensor data in simulation against real-world data
- Create multiple simulation scenarios to test different conditions