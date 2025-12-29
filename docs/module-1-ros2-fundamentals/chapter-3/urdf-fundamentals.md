---
sidebar_position: 2
---

# URDF Fundamentals

## Overview

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including links, joints, and their relationships.

## URDF Structure

A URDF file typically contains:
- **Links**: Rigid parts of the robot (e.g., chassis, wheels, arms)
- **Joints**: Connections between links (e.g., rotational, prismatic, fixed)
- **Visual**: How the robot appears in simulation
- **Collision**: How the robot interacts with the environment
- **Inertial**: Mass, center of mass, and inertia properties

## Basic URDF Example

Here's a simple URDF for a differential drive robot:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
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
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 0" rpy="1.5708 0 0"/>
  </joint>
</robot>
```

## Links

Links represent rigid bodies in the robot. Each link has:
- A name
- Visual properties (how it looks)
- Collision properties (how it interacts)
- Inertial properties (mass and inertia)

## Joints

Joints define the relationship between links. Common joint types:
- **Fixed**: No movement between links
- **Revolute**: Rotational movement around an axis
- **Continuous**: Like revolute but unlimited rotation
- **Prismatic**: Linear movement along an axis
- **Floating**: 6 degrees of freedom

## Working with URDF in ROS 2

In ROS 2, URDF files are typically loaded using the `robot_state_publisher` node, which reads the URDF and publishes the appropriate transforms to the `/tf` topic.

## Best Practices

- Use meaningful names for links and joints
- Ensure proper parent-child relationships
- Include both visual and collision properties
- Use appropriate inertial properties for physics simulation
- Keep URDF files organized and well-commented