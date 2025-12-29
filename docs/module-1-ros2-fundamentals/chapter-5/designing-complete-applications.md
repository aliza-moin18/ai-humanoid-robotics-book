---
sidebar_position: 2
---

# Designing Complete ROS 2 Applications

## Overview

Designing complete ROS 2 applications requires understanding how to structure complex systems with multiple nodes, coordinate communication between components, and manage the overall system lifecycle. This section covers the principles and patterns for building robust ROS 2 applications.

## Application Architecture Patterns

### Component-Based Architecture

In ROS 2, complex applications are typically built using a component-based architecture where each node performs a specific function:

- **Perception nodes**: Process sensor data to understand the environment
- **Planning nodes**: Determine appropriate actions based on goals and current state
- **Control nodes**: Execute actions by sending commands to actuators
- **Monitoring nodes**: Track system state and performance

### Layered Architecture

Applications often follow a layered approach:

1. **Hardware Abstraction Layer**: Direct interface with sensors and actuators
2. **Driver Layer**: ROS 2 interfaces to hardware components
3. **Middleware Layer**: Communication between components
4. **Application Layer**: High-level logic and behavior

## Design Principles

### Modularity

Each node should have a single, well-defined responsibility:

```python
# Good: Single responsibility
class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(PointCloud, 'point_cloud', 10)

    def scan_callback(self, msg):
        # Process scan data into point cloud
        processed_data = self.process_scan(msg)
        self.publisher.publish(processed_data)
```

### Loose Coupling

Nodes should communicate through well-defined interfaces (topics, services, actions) rather than direct function calls:

```python
# Good: Loose coupling through topics
class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation')
        # Subscribe to sensor data
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        # Publish navigation commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
```

### High Cohesion

Related functionality should be grouped together within nodes:

```python
# Good: High cohesion - all localization functions in one node
class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'pose', 10)
        
    def map_callback(self, msg):
        # Handle map data
        pass
        
    def odom_callback(self, msg):
        # Handle odometry data
        pass
        
    def scan_callback(self, msg):
        # Handle scan data for localization
        pass
```

## System Design Process

### 1. Requirements Analysis

Identify what the robot needs to do:
- Functional requirements (what the system should do)
- Non-functional requirements (performance, safety, reliability)

### 2. Component Identification

Break down the system into logical components:
- What data does each component need?
- What data does each component produce?
- What services does each component provide?

### 3. Interface Definition

Define the communication interfaces between components:
- Topics: asynchronous data flow
- Services: synchronous request/response
- Actions: long-running tasks with feedback

### 4. System Architecture

Create a high-level architecture diagram showing:
- Node relationships
- Communication patterns
- Data flow

### 5. Implementation Planning

Plan the implementation in phases:
- Core functionality first
- Error handling and recovery
- Performance optimization
- Testing and validation

## Launch System for Complex Applications

Use launch files to manage complex multi-node systems:

```python
# launch/complete_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch argument
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # Perception node
    perception_node = Node(
        package='my_robot_perception',
        executable='perception_node',
        name='perception',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/camera/image_raw', '/sim_camera/image_raw'),
        ]
    )
    
    # Planning node
    planning_node = Node(
        package='my_robot_planning',
        executable='planner_node',
        name='planner',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Control node
    control_node = Node(
        package='my_robot_control',
        executable='controller_node',
        name='controller',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Return the launch description
    return LaunchDescription([
        declare_use_sim_time,
        perception_node,
        planning_node,
        control_node,
    ])
```

## Best Practices for Application Design

1. **Use Composition When Appropriate**: For tightly coupled components, consider using composition instead of separate processes
2. **Implement Proper Error Handling**: Design for failure scenarios and implement recovery strategies
3. **Monitor System Health**: Implement monitoring and logging for debugging and maintenance
4. **Design for Testability**: Structure nodes so they can be tested in isolation
5. **Consider Performance**: Optimize communication patterns and processing for your specific use case

## Example: Complete Navigation Application

Let's look at how these principles come together in a complete navigation application:

```python
# navigation_system.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class NavigationSystem(Node):
    def __init__(self):
        super().__init__('navigation_system')
        
        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, 'navigation_active', 10)
        
        # Internal state
        self.current_pose = None
        self.goal_pose = None
        self.obstacle_detected = False
        self.navigation_active = False
        
        # Timer for main control loop
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def odom_callback(self, msg):
        # Update current pose
        self.current_pose = msg.pose.pose
    
    def scan_callback(self, msg):
        # Check for obstacles
        self.obstacle_detected = min(msg.ranges) < 0.5  # 0.5m threshold
    
    def goal_callback(self, msg):
        # Set new goal
        self.goal_pose = msg.pose
        self.navigation_active = True
        self.status_pub.publish(Bool(data=True))
    
    def control_loop(self):
        if not self.navigation_active or not self.current_pose or not self.goal_pose:
            return
            
        if self.obstacle_detected:
            # Stop if obstacle detected
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
            
        # Calculate and execute navigation commands
        # (Implementation would go here)
        pass
```

This example demonstrates how to structure a complete navigation application using ROS 2 principles.