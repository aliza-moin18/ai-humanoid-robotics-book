---
sidebar_position: 1
---

# Section 3.1: URDF and SDF Model Creation

## Overview

This section covers the creation and configuration of robot models using both URDF (Unified Robot Description Format) and SDF (Simulation Description Format) in Gazebo. Students will learn the differences between these formats, when to use each, and how to create detailed humanoid robot models.

## URDF vs SDF: Key Differences

### URDF (Unified Robot Description Format)
- **Purpose**: Primarily for robot structure and kinematics
- **Origin**: Developed for ROS ecosystem
- **Strengths**: Excellent for kinematic chains, widely supported in ROS
- **Limitations**: Limited physics and simulation features
- **Best for**: Robot description, kinematic modeling, ROS integration

### SDF (Simulation Description Format)
- **Purpose**: Comprehensive simulation description
- **Origin**: Developed for Gazebo simulation
- **Strengths**: Rich physics, sensors, plugins, and simulation features
- **Limitations**: More complex, primarily Gazebo-focused
- **Best for**: Complete simulation scenarios, physics modeling

## URDF Model Structure

### Basic URDF Template

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <inertial>
      <mass value="1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Joints connect links -->
  <joint name="base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 1"/>
  </joint>

  <link name="child_link">
    <inertial>
      <mass value="1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>
</robot>
```

### URDF Elements for Humanoid Robots

#### Links
- Define rigid bodies with mass, inertia, visual, and collision properties
- For humanoid robots: head, torso, arms, legs, feet

#### Joints
- Connect links with specific degrees of freedom
- Types: revolute, continuous, prismatic, fixed, floating, planar
- For humanoid robots: typically revolute joints for articulation

#### Materials
- Define visual appearance properties
- Colors, textures, and shading parameters

## SDF Model Structure

### Basic SDF Template

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="humanoid_robot">
    <pose>0 0 0 0 0 0</pose>

    <!-- Links -->
    <link name="base_link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>

      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
      </visual>

      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
      </collision>
    </link>

    <!-- Joints -->
    <joint name="base_joint" type="fixed">
      <parent>base_link</parent>
      <child>child_link</child>
      <pose>0 0 1 0 0 0</pose>
    </joint>

    <link name="child_link">
      <pose>0 0 1 0 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```

## Creating Humanoid Robot Models

### URDF Model for Humanoid Robot

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <inertial>
      <mass value="20"/>
      <origin xyz="0 0 0.2"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.2"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.2"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.2"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="3"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Left upper arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="2"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>
</robot>
```

### SDF Model for Humanoid Robot

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_humanoid_sdf">
    <pose>0 0 0 0 0 0</pose>

    <!-- Torso -->
    <link name="torso">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>20.0</mass>
        <inertia>
          <ixx>0.5</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.5</iyy>
          <iyz>0.0</iyz>
          <izz>0.2</izz>
        </inertia>
      </inertial>
      <visual name="torso_visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.4</size>
          </box>
        </geometry>
      </visual>
      <collision name="torso_collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.4</size>
          </box>
        </geometry>
      </collision>
    </link>

    <!-- Head -->
    <link name="head">
      <pose>0 0 0.4 0 0 0</pose>
      <inertial>
        <mass>3.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.01</iyy>
          <iyz>0.0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <visual name="head_visual">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </visual>
      <collision name="head_collision">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
      </collision>
    </link>

    <!-- Neck joint -->
    <joint name="neck_joint" type="revolute">
      <parent>torso</parent>
      <child>head</child>
      <pose>0 0 0.4 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.5</lower>
          <upper>0.5</upper>
          <effort>100</effort>
          <velocity>1</velocity>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
```

## Converting Between URDF and SDF

### URDF to SDF Conversion

```bash
gz sdf -p robot.urdf > robot.sdf
```

### SDF to URDF Conversion

SDF to URDF conversion is not directly supported. Instead, use ROS tools if available:

```bash
# Using ROS tools (requires ROS installation)
rosrun xacro xacro robot.urdf.xacro > robot.urdf
```

## Advanced Model Features

### Gazebo-Specific Extensions in URDF

To add Gazebo-specific features to a URDF model:

```xml
<robot name="humanoid_with_gazebo_extensions">
  <!-- Standard URDF content -->
  <link name="sensor_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Gazebo-specific content -->
  <gazebo reference="sensor_link">
    <material>Gazebo/Blue</material>
    <turnGravityOff>false</turnGravityOff>
  </gazebo>

  <!-- Adding a sensor -->
  <gazebo>
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
      </imu>
    </sensor>
  </gazebo>
</robot>
```

## Model Validation

### URDF Validation

```bash
# Check URDF syntax
check_urdf robot.urdf

# Parse URDF and show information
urdf_parser_test robot.urdf
```

### SDF Validation

```bash
# Check SDF syntax
gz sdf -k robot.sdf

# Parse SDF and show information
gz sdf -p robot.sdf
```

## Best Practices

1. **Use Meaningful Names**: Use descriptive names for links and joints
2. **Consistent Units**: Use SI units consistently (meters, kilograms, seconds)
3. **Realistic Properties**: Base mass and inertia on real-world values
4. **Modular Design**: Organize models for easy modification and reuse
5. **Documentation**: Comment complex models for future reference
6. **Validation**: Regularly validate models using appropriate tools

## Troubleshooting Common Issues

### Issue 1: Model Fails to Load
- **Check**: Syntax errors in XML
- **Verify**: All referenced files exist (meshes, textures)
- **Validate**: Use appropriate validation tools

### Issue 2: Physics Issues
- **Check**: Mass and inertia values
- **Verify**: Joint limits and types
- **Adjust**: Collision geometry alignment

### Issue 3: Visual Issues
- **Check**: Visual geometry alignment with collision geometry
- **Verify**: Material properties and textures
- **Adjust**: Scaling and positioning

## Next Steps

After learning about URDF and SDF model creation, proceed to [Section 3.2: Physics Parameters in Gazebo](../chapter-3/section-3-2-physics-parameters) to explore advanced physics configuration in Gazebo.