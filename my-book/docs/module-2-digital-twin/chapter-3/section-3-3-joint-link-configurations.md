---
sidebar_position: 3
---

# Section 3.3: Joint and Link Configurations

## Overview

This section focuses on configuring joints and links in Gazebo for humanoid robot simulation. Students will learn about different joint types, constraint configurations, and advanced link properties that affect robot behavior and simulation accuracy.

## Joint Types in Gazebo

### Fixed Joint
- **Purpose**: Connects two links without allowing relative motion
- **Use Case**: Attaching sensors, combining rigid parts
- **Configuration**:

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>
```

### Revolute Joint
- **Purpose**: Allows rotation around a single axis
- **Use Case**: Most humanoid robot joints (elbows, knees, wrists)
- **Configuration**:

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotation axis -->
  <limit lower="-2.5" upper="1.5" effort="50" velocity="3"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

### Continuous Joint
- **Purpose**: Allows unlimited rotation around a single axis
- **Use Case**: Rotating wheels, continuous rotation joints
- **Configuration**:

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base"/>
  <child link="rotor"/>
  <axis xyz="0 0 1"/>
  <dynamics damping="0.1"/>
</joint>
```

### Prismatic Joint
- **Purpose**: Allows linear motion along a single axis
- **Use Case**: Linear actuators, telescoping mechanisms
- **Configuration**:

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="base"/>
  <child link="slider"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="100" velocity="1"/>
  <dynamics damping="1.0"/>
</joint>
```

### Floating Joint
- **Purpose**: Allows 6 degrees of freedom (3 translation, 3 rotation)
- **Use Case**: Base of floating robots, unconstrained objects
- **Configuration**:

```xml
<joint name="floating_joint" type="floating">
  <parent link="world"/>
  <child link="floating_object"/>
</joint>
```

## Joint Configuration Parameters

### Joint Limits

For revolute and prismatic joints:

```xml
<joint name="limited_joint" type="revolute">
  <limit 
    lower="-1.57"      <!-- Lower limit in radians -->
    upper="1.57"       <!-- Upper limit in radians -->
    effort="100"       <!-- Maximum effort (N or Nm) -->
    velocity="2"       <!-- Maximum velocity (m/s or rad/s) -->
    stiffness="10000"  <!-- Spring stiffness (not commonly used) -->
    dissipation="1"    <!-- Spring damping (not commonly used) -->
  />
</joint>
```

### Joint Dynamics

Damping and friction parameters:

```xml
<joint name="damped_joint" type="revolute">
  <dynamics 
    damping="1.0"    <!-- Viscous damping coefficient -->
    friction="0.5"   <!-- Static friction coefficient -->
    spring_reference="0.0"    <!-- Spring reference angle/position -->
    spring_stiffness="0.0"    <!-- Spring stiffness -->
  />
</joint>
```

## Advanced Joint Configurations

### Mimic Joints

For coupled joint movements (e.g., two hands moving together):

```xml
<joint name="right_elbow" type="revolute">
  <parent link="upper_arm_right"/>
  <child link="lower_arm_right"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="1.0" effort="50" velocity="3"/>
  <mimic joint="left_elbow" multiplier="1.0" offset="0.0"/>
</joint>
```

### Gearbox Joints

For complex transmission systems:

```xml
<joint name="gearbox_joint" type="revolute">
  <parent link="motor"/>
  <child link="output_shaft"/>
  <axis xyz="0 0 1"/>
  <transmission>
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="gearbox_joint">
      <hardwareInterface>PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor_actuator">
      <mechanicalReduction>10</mechanicalReduction>  <!-- 10:1 gear ratio -->
    </actuator>
  </transmission>
</joint>
```

## Link Configuration Parameters

### Inertial Properties

Critical for realistic physics simulation:

```xml
<link name="arm_link">
  <inertial>
    <mass value="1.5"/>  <!-- Mass in kg -->
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>  <!-- Center of mass offset -->
    <inertia 
      ixx="0.01" ixy="0" ixz="0" 
      iyy="0.01" iyz="0" 
      izz="0.002"/>  <!-- Inertia tensor values -->
  </inertial>
</link>
```

### Visual Properties

For rendering and visualization:

```xml
<link name="visual_link">
  <visual name="main_visual">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://robot_description/meshes/arm.dae"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
</link>
```

### Collision Properties

For physics interaction:

```xml
<link name="collision_link">
  <collision name="main_collision">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>0.8</mu>
          <mu2>0.8</mu2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.1</restitution_coefficient>
      </bounce>
    </surface>
  </collision>
</link>
```

## Humanoid Robot Joint Configurations

### Leg Joint Configuration

Example configuration for a humanoid leg with 6 DOF:

```xml
<!-- Hip joint (3 DOF using 3 single-DOF joints) -->
<joint name="hip_yaw" type="revolute">
  <parent link="torso"/>
  <child link="hip"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="200" velocity="2"/>
  <dynamics damping="2.0" friction="0.5"/>
</joint>

<joint name="hip_roll" type="revolute">
  <parent link="hip"/>
  <child link="thigh"/>
  <axis xyz="1 0 0"/>
  <limit lower="-0.4" upper="1.0" effort="200" velocity="2"/>
  <dynamics damping="2.0" friction="0.5"/>
</joint>

<joint name="hip_pitch" type="revolute">
  <parent link="thigh"/>
  <child link="calf"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.5" upper="0.5" effort="200" velocity="2"/>
  <dynamics damping="2.0" friction="0.5"/>
</joint>

<!-- Knee joint -->
<joint name="knee" type="revolute">
  <parent link="calf"/>
  <child link="foot"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.5" effort="200" velocity="2"/>
  <dynamics damping="1.5" friction="0.5"/>
</joint>
```

### Arm Joint Configuration

Example configuration for a humanoid arm:

```xml
<!-- Shoulder joint (3 DOF) -->
<joint name="shoulder_abduction" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.7" upper="1.5" effort="100" velocity="3"/>
  <dynamics damping="1.0" friction="0.2"/>
</joint>

<joint name="shoulder_flexion" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <axis xyz="0 0 1"/>
  <limit lower="-2.3" upper="1.0" effort="100" velocity="3"/>
  <dynamics damping="1.0" friction="0.2"/>
</joint>

<joint name="elbow" type="revolute">
  <parent link="lower_arm"/>
  <child link="hand"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="2.3" effort="50" velocity="4"/>
  <dynamics damping="0.8" friction="0.1"/>
</joint>
```

## Joint Safety and Control Parameters

### Safety Controllers

For preventing joint damage:

```xml
<joint name="safe_joint" type="revolute">
  <safety_controller 
    k_position="100"     <!-- Position control gain -->
    k_velocity="10"      <!-- Velocity control gain -->
    soft_lower_limit="-1.5"  <!-- Soft lower limit -->
    soft_upper_limit="1.5"/>  <!-- Soft upper limit -->
</joint>
```

### Control Interfaces

For ROS integration:

```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  </plugin>
</gazebo>
```

## Complex Joint Systems

### Universal Joint (Simulation)

Using two revolute joints to simulate a universal joint:

```xml
<joint name="universal_joint_1" type="revolute">
  <parent link="input_shaft"/>
  <child link="intermediate_part"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="3"/>
</joint>

<joint name="universal_joint_2" type="revolute">
  <parent link="intermediate_part"/>
  <child link="output_shaft"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="3"/>
</joint>
```

### Ball Joint (Simulation)

Using three revolute joints to simulate a ball joint:

```xml
<joint name="ball_joint_yaw" type="revolute">
  <parent link="base"/>
  <child link="ball_part_1"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="3"/>
</joint>

<joint name="ball_joint_pitch" type="revolute">
  <parent link="ball_part_1"/>
  <child link="ball_part_2"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="3"/>
</joint>

<joint name="ball_joint_roll" type="revolute">
  <parent link="ball_part_2"/>
  <child link="end_effector"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="3"/>
</joint>
```

## Validation and Testing

### Joint Configuration Validation

Test joint behavior with simple movements:

```xml
<!-- Test model with single joint -->
<robot name="joint_test">
  <link name="base">
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>
  
  <link name="moving_part">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.1 0.1 0.3"/>
      </geometry>
    </visual>
  </link>
  
  <joint name="test_joint" type="revolute">
    <parent link="base"/>
    <child link="moving_part"/>
    <origin xyz="0 0 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

### Link Configuration Validation

Verify mass properties and collisions:

```xml
<!-- Validation model -->
<robot name="link_validation">
  <link name="test_link">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.2"/>
      </geometry>
    </collision>
  </link>
</robot>
```

## Best Practices

1. **Realistic Joint Limits**: Base limits on real robot capabilities
2. **Proper Damping**: Use appropriate damping values for stability
3. **Consistent Units**: Use SI units throughout configurations
4. **Validation Testing**: Test joints individually before integration
5. **Documentation**: Comment complex joint configurations
6. **Safety Margins**: Include safety margins in joint limits
7. **Symmetry**: Maintain left/right symmetry in humanoid robots

## Troubleshooting Common Issues

### Issue 1: Joint Limit Violations
- **Check**: Joint limit values and controller behavior
- **Verify**: Controller command ranges
- **Adjust**: Add safety margins to limits

### Issue 2: Unstable Joint Behavior
- **Check**: Joint damping and friction values
- **Verify**: Mass distribution and inertia tensors
- **Adjust**: Physics parameters (time step, solver iterations)

### Issue 3: Unexpected Joint Coupling
- **Check**: Multiple joint constraints between same links
- **Verify**: Joint axes and orientations
- **Adjust**: Joint configuration to match intended DOF

## Next Steps

After configuring joints and links for humanoid robots, proceed to [Section 3.4: Environmental Physics Simulation](../chapter-3/section-3-4-environmental-physics) to learn about simulating environmental physics in Gazebo.