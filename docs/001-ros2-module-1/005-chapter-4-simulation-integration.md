# Chapter 4: Integration with Simulation

## Overview

This chapter focuses on integrating ROS 2 with simulation environments, particularly Gazebo. You'll learn how to connect ROS 2 nodes to simulated robots, perform testing and debugging in simulation, and understand the transition from simulation to real hardware.

## Learning Objectives

After completing this chapter, you will be able to:
- Set up and configure Gazebo simulation environments
- Integrate ROS 2 nodes with Gazebo for robot simulation
- Use debugging and visualization tools in simulation
- Understand the process of transitioning from simulation to hardware

## 4.1 ROS 2 and Gazebo Integration

### Introduction to Gazebo

Gazebo is a 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It's widely used for robotics research and development, allowing developers to test algorithms and robot behaviors in a safe, repeatable environment.

### Gazebo and ROS 2 Architecture

ROS 2 and Gazebo integrate through:
- Gazebo plugins that interface with ROS 2 topics and services
- TF2 transforms for robot state broadcasting
- Standard ROS 2 message types for sensor and actuator data
- Launch files to start both Gazebo and ROS 2 nodes simultaneously

### Installing Gazebo and ROS 2 Integration

The recommended setup is to install Gazebo alongside ROS 2 Humble Hawksbill. The ROS 2 package `ros-humble-gazebo-ros-pkgs` provides the necessary integration tools.

### Basic Gazebo ROS 2 Integration

To spawn a robot model in Gazebo with ROS 2 control, you typically need:

1. A URDF model of the robot
2. Robot state publisher
3. Controller configuration
4. Gazebo plugins for physics and ROS interface

#### Example Gazebo Launch File

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get Gazebo launch file
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])
    
    # Include Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'world': PathJoinSubstitution([FindPackageShare('my_robot_gazebo'), 'worlds', 'my_world.sdf'])
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': PathJoinSubstitution([
            FindPackageShare('my_robot_description'), 'urdf', 'my_robot.urdf'
        ])}]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

## 4.2 Testing with Simulated Robots

### Creating a Simulation Scenario

Testing with simulated robots allows for rapid iteration and development without risk to hardware. A typical simulation scenario includes:

1. **Environment Setup**: Configure the Gazebo world with relevant objects
2. **Robot Spawn**: Load and position the robot model
3. **Sensor Configuration**: Set up sensors to match real hardware
4. **Control Interface**: Implement ROS 2 interfaces for commanding the robot
5. **Data Collection**: Log sensor data and robot states for analysis

### Example: Simulated Navigation Test

Here's a complete example of a navigation test in simulation:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class SimulationNavigator(Node):
    def __init__(self):
        super().__init__('simulation_navigator')
        
        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscriber = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, 10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        
        # Timer for navigation loop
        self.timer = self.create_timer(0.1, self.navigation_loop)
        
        # Robot state
        self.current_position = [0.0, 0.0]
        self.current_yaw = 0.0
        self.scan_data = None
        
        # Navigation goal
        self.goal_x = 5.0
        self.goal_y = 5.0
        
    def laser_callback(self, msg):
        self.scan_data = msg.ranges
        
    def odom_callback(self, msg):
        # Extract position
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        
        # Extract orientation (convert quaternion to yaw)
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    
    def navigation_loop(self):
        if self.scan_data is None:
            return
            
        # Simple obstacle avoidance based on laser scan
        min_distance = min(self.scan_data[300:420])  # Front 120 degrees
        
        # Calculate direction to goal
        dx = self.goal_x - self.current_position[0]
        dy = self.goal_y - self.current_position[1]
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        
        # Create twist message
        twist = Twist()
        
        if min_distance < 1.0:  # Obstacle too close
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn right
        elif distance_to_goal > 0.5:  # Not at goal yet
            # Move toward goal with proportional control
            twist.linear.x = min(0.5, distance_to_goal * 0.2)
            twist.angular.z = math.atan2(dy, dx) - self.current_yaw
        else:
            # At goal, stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(twist)
        
        # Log progress
        self.get_logger().info(
            f'Position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}), '
            f'Distance to goal: {distance_to_goal:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    navigator = SimulationNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Automated Testing in Simulation

You can also create automated tests that run in simulation:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from example_interfaces.action import Fibonacci
import time

class SimulationTestSuite(Node):
    def __init__(self):
        super().__init__('simulation_test_suite')
        
        # Test publisher
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Test subscribers
        self.odom_sub = self.create_subscription(
            Float64, 'test_result', self.test_result_callback, 10
        )
        
        # Test result tracking
        self.test_results = {}
        
    def test_result_callback(self, msg):
        # Process test results
        pass
        
    def run_movement_test(self):
        """Test robot movement capabilities"""
        self.get_logger().info("Running movement test...")
        
        # Send command to move forward
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        
        # Send command for 2 seconds
        start_time = time.time()
        while time.time() - start_time < 2.0:
            self.vel_publisher.publish(twist)
            time.sleep(0.1)
        
        # Stop robot
        twist.linear.x = 0.0
        self.vel_publisher.publish(twist)
        
        self.get_logger().info("Movement test completed")
        
    def run_sensor_test(self):
        """Test sensor functionality"""
        self.get_logger().info("Running sensor test...")
        
        # Implementation for sensor testing
        # This would involve publishing commands and verifying sensor feedback
        
        self.get_logger().info("Sensor test completed")
        
    def run_all_tests(self):
        """Run all simulation tests"""
        self.get_logger().info("Starting simulation test suite...")
        
        # Run each test
        self.run_movement_test()
        self.run_sensor_test()
        
        # Add more tests here
        
        self.get_logger().info("All tests completed!")

def main(args=None):
    rclpy.init(args=args)
    
    test_suite = SimulationTestSuite()
    
    # Run all tests
    test_suite.run_all_tests()
    
    test_suite.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4.3 Debugging and Visualization

### Visualization Tools in ROS 2

Several tools help visualize robot state and behavior in simulation:

#### RViz2
RViz2 is the 3D visualization tool for ROS 2. It can display:
- Robot models (through TF and robot_description)
- Sensor data (lasers, cameras, point clouds)
- Path planning results
- Map data
- Trajectory information

Example RViz2 configuration node:
```python
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        
        # Publisher for visualization markers
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Timer to update visualization
        self.timer = self.create_timer(0.5, self.update_visualization)
        
    def update_visualization(self):
        # Create a marker for visualization
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set marker properties
        marker.scale.x = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Add points to the marker (this would come from robot path)
        p1 = Point()
        p1.x = 0.0
        p1.y = 0.0
        p1.z = 0.0
        marker.points.append(p1)
        
        p2 = Point()
        p2.x = 1.0
        p2.y = 1.0
        p2.z = 0.0
        marker.points.append(p2)
        
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    
    viz_node = VisualizationNode()
    
    try:
        rclpy.spin(viz_node)
    except KeyboardInterrupt:
        pass
    
    viz_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### PlotJuggler
PlotJuggler allows real-time plotting of ROS 2 topics, useful for debugging sensor data, control signals, and other numerical data.

### Common Simulation Debugging Techniques

1. **Topic Monitoring**: Use `ros2 topic echo` to monitor data flow
2. **Transform Debugging**: Use `ros2 run tf2_tools view_frames` to visualize frame relationships
3. **Log Analysis**: Monitor node logs for errors and warnings
4. **Physics Debugging**: Visualize collision shapes and contacts in Gazebo
5. **Performance Monitoring**: Check simulation real-time factor (RTF)

### Debugging Example: Collision Detection

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import math

class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        
        # Publishers
        self.collision_pub = self.create_publisher(Bool, 'collision_warning', 10)
        self.closest_point_pub = self.create_publisher(PointStamped, 'closest_object', 10)
        
    def scan_callback(self, msg):
        # Find closest object
        min_distance = float('inf')
        min_index = -1
        
        for i, distance in enumerate(msg.ranges):
            if not math.isnan(distance) and distance < min_distance:
                min_distance = distance
                min_index = i
        
        # Publish collision warning if needed
        warning_msg = Bool()
        warning_msg.data = min_distance < 0.5  # Collision if closer than 0.5m
        self.collision_pub.publish(warning_msg)
        
        # Publish closest point if found
        if min_index >= 0 and min_distance < float('inf'):
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = msg.header.frame_id
            
            # Convert polar to Cartesian
            angle = msg.angle_min + min_index * msg.angle_increment
            point_msg.point.x = min_distance * math.cos(angle)
            point_msg.point.y = min_distance * math.sin(angle)
            point_msg.point.z = 0.0
            
            self.closest_point_pub.publish(point_msg)
        
        self.get_logger().info(f'Closest obstacle: {min_distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    
    detector = CollisionDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4.4 Simulation to Hardware Transition

### Key Considerations

Transitioning from simulation to hardware requires careful planning:

1. **Model Accuracy**: Ensure the simulation model accurately represents the real robot
2. **Sensor Differences**: Real sensors may have different noise patterns, latency, and accuracy
3. **Control Frequency**: Real-time constraints may differ from simulation
4. **Environmental Differences**: Lighting, surfaces, and dynamics differ from simulation
5. **Timing and Synchronization**: Real-time systems have different timing characteristics

### Hardware-in-the-Loop Testing

This approach uses real hardware components with simulated environment:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class HardwareSimulationBridge(Node):
    def __init__(self):
        super().__init__('hardware_simulation_bridge')
        
        # Publishers to hardware
        self.joint_command_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )
        
        # Subscribers from hardware
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        
        # Robot state tracking
        self.current_joint_positions = {}
        
    def joint_state_callback(self, msg):
        # Update current joint positions
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
    
    def control_loop(self):
        # This is where you would implement your control algorithm
        # reading from real hardware and potentially interacting with simulation
        pass

def main(args=None):
    rclpy.init(args=args)
    
    bridge = HardwareSimulationBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Verification and Validation Strategies

Before moving to hardware:

1. **Parameter Tuning**: Adjust controllers in simulation to match hardware characteristics
2. **Limit Testing**: Test at boundary conditions in simulation
3. **Failure Mode Testing**: Verify how the system behaves during failures
4. **Safety Checks**: Implement safety mechanisms that work on both simulation and hardware

### Hardware Launch Configuration

When transitioning to hardware, you'll often need different launch files:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Argument to switch between simulation and hardware
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Robot driver node
    robot_driver = Node(
        package='my_robot_driver',
        executable='robot_driver',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_driver,
        robot_state_publisher
    ])
```

## Summary

This chapter covered the integration of ROS 2 with simulation environments:
- How to connect ROS 2 nodes with Gazebo simulation
- Testing techniques and creating simulation scenarios
- Debugging and visualization tools for simulation
- The transition process from simulation to real hardware

Simulation is a powerful tool for robotics development, allowing for safe, repeatable testing of algorithms before deployment on real hardware. The integration between ROS 2 and Gazebo enables comprehensive testing and validation of robotic systems.

## Next Steps

Continue to [Chapter 5: Applied Robotics Projects](006-chapter-5-applied-projects.md) to learn how to design and implement complete ROS 2-based robot applications that integrate all the concepts covered in previous chapters.