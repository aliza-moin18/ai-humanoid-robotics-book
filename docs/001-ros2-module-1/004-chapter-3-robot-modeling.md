# Chapter 3: Robot Description and Modeling

## Overview

This chapter covers robot description in ROS 2, focusing on URDF (Unified Robot Description Format). You'll learn how to create, modify, and work with robot models for humanoid robots, including the use of the robot state publisher and sensor integration.

## Learning Objectives

After completing this chapter, you will be able to:
- Understand and create URDF files for robot description
- Create and modify robot models for humanoid robots
- Use the robot state publisher to broadcast transforms
- Integrate sensors into robot models

## 3.1 URDF Fundamentals

### What is URDF?

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robots. It contains information about joints, links, inertial properties, visual models, and collision models of a robot. URDF is essential for robot simulation, visualization, and motion planning.

### URDF Structure

A URDF file typically includes:
- **Links**: Rigid bodies with physical properties
- **Joints**: Connections between links
- **Visual elements**: How the robot appears in simulation
- **Collision elements**: How the robot interacts with its environment
- **Inertial properties**: Mass, center of mass, and inertia for physics simulation

### Basic URDF Example

Here's a simple URDF for a robot with two links connected by a joint:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Top link -->
  <link name="top_link">
    <visual>
      <geometry>
        <box size="0.4 0.4 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.4 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting base and top links -->
  <joint name="base_to_top" type="fixed">
    <parent link="base_link"/>
    <child link="top_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>
</robot>
```

### Common URDF Elements

#### Links
- `<link>`: Defines a rigid body
- Contains: `<visual>`, `<collision>`, `<inertial>`, `<material>`

#### Joints
- `<joint>`: Connects two links
- Types: `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`
- Contains: `<parent>`, `<child>`, `<origin>`, `<axis>`, `<limit>`

#### Materials
- `<material>`: Defines visual appearance
- Contains: `<color>` with rgba values

## 3.2 Creating Robot Models

### Designing a Humanoid Robot Model

Creating a humanoid robot model requires careful consideration of the kinematic structure. A typical humanoid has:
- A torso/upper body
- Two arms with shoulders, elbows, wrists
- Two legs with hips, knees, ankles
- A head/neck
- Joints connecting these parts

### Example: Simple Humanoid URDF

Here's a more complex example of a simple humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.0" upper="1.0" effort="100" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.8 0.8 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.8 0.8 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.36" effort="30" velocity="1"/>
  </joint>

  <!-- Similar definitions for right arm and legs would follow -->
</robot>
```

### URDF Best Practices

1. **Use descriptive names**: Use clear, consistent naming conventions
2. **Start simple**: Begin with basic shapes and add complexity gradually
3. **Validate regularly**: Use `check_urdf` tool to verify your URDF
4. **Use Xacro**: For complex models, use Xacro (XML Macros) to reduce repetition
5. **Include proper inertial properties**: Essential for physics simulation

## 3.3 Robot State Publisher

### Overview

The robot_state_publisher is a ROS 2 node that reads a URDF and broadcasts the position and orientation of each link to the tf2 transform system. This allows other ROS nodes to know the current configuration of the robot.

### How Robot State Publisher Works

1. Reads the robot description (URDF) from the parameter server
2. Subscribes to joint state messages on the `/joint_states` topic
3. Calculates the forward kinematics of the robot
4. Publishes the resulting transforms to the `/tf` or `/tf_static` topics

### Using Robot State Publisher

To launch the robot state publisher, you typically run:

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(find-pkg-share my_robot_description)/urdf/my_robot.urdf'
```

### Example Node with Robot State Publisher

Here's how to create a node that publishes joint states for the robot state publisher:

```python
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer to publish joint states at regular intervals
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Set up the joint state message
        self.joint_state = JointState()
        self.joint_state.name = ['left_shoulder_joint', 'left_elbow_joint', 
                                'right_shoulder_joint', 'right_elbow_joint']
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0, 0.0, 0.0]
        self.joint_state.effort = [0.0, 0.0, 0.0, 0.0]

    def publish_joint_states(self):
        # Update joint values (this could be from a sensor or simulation)
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.header.frame_id = 'base_link'
        
        # Simple example: make joints oscillate
        t = self.get_clock().now().nanoseconds / 1000000000.0
        self.joint_state.position[0] = math.sin(t)
        self.joint_state.position[1] = math.cos(t)
        self.joint_state.position[2] = math.sin(t + math.pi)
        self.joint_state.position[3] = math.cos(t + math.pi)
        
        # Publish the joint state
        self.joint_pub.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    
    joint_publisher = JointStatePublisher()
    
    try:
        rclpy.spin(joint_publisher)
    except KeyboardInterrupt:
        pass
    
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.4 Sensors and Plugins

### Integrating Sensors in URDF

Sensors are typically modeled as additional links connected to existing robot links. Common sensors include:

1. **IMU (Inertial Measurement Unit)**: Usually placed on the robot's torso
2. **LiDAR**: Often placed on top of the robot or on a rotating mount
3. **Cameras**: Can be placed on head, torso, or end-effectors

### Example: Adding a Camera to URDF

```xml
<!-- Add to existing robot -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.1 0.03"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.1 0.03"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for camera simulation -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_node">
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Gazebo Plugins

For simulation integration, URDF files can include Gazebo-specific tags:

- `<gazebo>`: Contains Gazebo-specific configuration
- `<material>`: Gazebo material properties
- `<collision>`: Gazebo collision properties
- `<sensor>`: Sensor simulation
- `<plugin>`: Gazebo plugins for physics, controllers, etc.

### Sensor Data Processing

Once sensors are integrated into your robot model, you'll need nodes to process the sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from cv_bridge import CvBridge
import cv2

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        
        # Initialize CvBridge for image processing
        self.bridge = CvBridge()
        
        # Subscribe to sensor topics
        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )
        
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Process the image (e.g., detect objects, etc.)
            self.get_logger().info(f'Received image of size: {cv_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def imu_callback(self, msg):
        # Process IMU data
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration
        
        self.get_logger().info(
            f'IMU - Orientation: ({orientation.x}, {orientation.y}, {orientation.z}, {orientation.w})'
        )
    
    def laser_callback(self, msg):
        # Process laser scan data
        ranges = msg.ranges
        min_range = min(ranges) if ranges else float('inf')
        
        self.get_logger().info(f'Laser - Min range: {min_range:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    
    sensor_processor = SensorProcessor()
    
    try:
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        pass
    
    sensor_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

This chapter covered robot description and modeling in ROS 2:
- URDF fundamentals for describing robot structure
- Creating complex robot models for humanoid robots
- Using the robot state publisher for transform broadcasting
- Integrating sensors into robot models with Gazebo plugins

Robot description is fundamental to robotics applications, enabling simulation, visualization, and motion planning. Understanding URDF and the robot state publisher is crucial for working with complex robotic systems.

## Next Steps

Continue to [Chapter 4: Integration with Simulation](005-chapter-4-simulation-integration.md) to learn about connecting ROS 2 with Gazebo for robot simulation.