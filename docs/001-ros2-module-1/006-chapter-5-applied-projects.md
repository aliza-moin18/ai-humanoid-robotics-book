# Chapter 5: Applied Robotics Projects

## Overview

This chapter integrates all the concepts learned in previous chapters into comprehensive, applied robotics projects. You'll learn how to design complete ROS 2-based robot applications, integrate perception, planning and control systems, consider performance optimizations, and implement best practices for robot software development.

## Learning Objectives

After completing this chapter, you will be able to:
- Design and implement complete ROS 2-based robot applications
- Integrate perception, planning, and control systems effectively
- Consider performance implications in robot software design
- Apply best practices for robust robot software development

## 5.1 Designing Complete ROS 2 Applications

### System Architecture Design

Designing a complete ROS 2 application requires a well-thought-out architecture that considers:

1. **Modularity**: Separate functionality into distinct nodes
2. **Communication**: Design efficient message passing between nodes
3. **Resource Management**: Consider computational and memory requirements
4. **Fault Tolerance**: Plan for graceful degradation when components fail
5. **Scalability**: Design to allow for additional features and capabilities

### Example: Complete Mobile Manipulation System

Let's design a complete mobile manipulation system that combines navigation and manipulation:

#### System Components

The system includes:
- **Navigation stack**: For mobile base movement
- **Manipulation stack**: For arm control
- **Perception stack**: For object recognition and localization
- **Task planner**: For high-level mission planning
- **State machine**: For coordinating different behaviors
- **Human-robot interface**: For user interaction

#### Architecture Diagram

```
+-------------------+    +---------------------+    +------------------+
|                   |    |                     |    |                  |
|  HRI Interface    |    |   Task Planner      |    |  State Machine   |
|                   |    |                     |    |                  |
+--------+----------+    +----------+----------+    +---------+--------+
         |                        |                         |
         +------------------------+-------------------------+
                                  |
               +------------------v------------------+
               |                                     |
               |            Mission Coordinator      |
               |                                     |
               +------------------+------------------+
                                  |
        +-------------------------v-------------------------+
        |                                                   |
        |                System Integrator                  |
        |                                                   |
        +------------------+------------------+-------------+
        |                  |                  |             |
+-------v-------+  +-------v-------+  +-------v-----+ +-----v--------+
| Navigation    |  | Manipulation  |  | Perception  | | Monitoring   |
| Stack         |  | Stack         |  | Stack       | | Tools        |
+---------------+  +---------------+  +-------------+ +--------------+
```

### Implementation Example: Mission Coordinator Node

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from builtin_interfaces.msg import Duration

class MissionCoordinator(Node):
    def __init__(self):
        super().__init__('mission_coordinator')
        
        # Initialize action clients
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self.moveit_client = ActionClient(
            self, MoveGroup, 'move_group'
        )
        
        # Subscribers for system state
        self.status_sub = self.create_subscription(
            String, 'system_status', self.status_callback, 10
        )
        
        # Publisher for mission commands
        self.command_pub = self.create_publisher(
            String, 'mission_commands', 10
        )
        
        # Timer to run mission logic
        self.timer = self.create_timer(1.0, self.mission_logic)
        
        # Mission state
        self.current_mission_step = 0
        self.system_status = "IDLE"
        self.mission_queue = [
            {"type": "navigate", "target": [1.0, 2.0, 0.0]},
            {"type": "manipulate", "object": "red_box"},
            {"type": "navigate", "target": [0.0, 0.0, 0.0]},
            {"type": "deposit", "location": "drop_zone"}
        ]
    
    def status_callback(self, msg):
        self.system_status = msg.data
    
    def mission_logic(self):
        if self.current_mission_step >= len(self.mission_queue):
            self.get_logger().info("Mission completed!")
            return
            
        if self.system_status != "READY":
            return  # Wait for system to be ready
            
        current_task = self.mission_queue[self.current_mission_step]
        
        if current_task["type"] == "navigate":
            self.execute_navigation_task(current_task["target"])
        elif current_task["type"] == "manipulate":
            self.execute_manipulation_task(current_task["object"])
        elif current_task["type"] == "deposit":
            self.execute_deposition_task(current_task["location"])
    
    def execute_navigation_task(self, target):
        self.get_logger().info(f"Starting navigation to {target}")
        
        # Wait for action server
        self.nav_client.wait_for_server()
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = target[0]
        goal_msg.pose.pose.position.y = target[1]
        
        # Calculate orientation to point toward next waypoint
        if self.current_mission_step < len(self.mission_queue) - 1:
            next_target = self.mission_queue[self.current_mission_step + 1]["target"]
            angle = math.atan2(
                next_target[1] - target[1],
                next_target[0] - target[0]
            )
        else:
            angle = 0.0  # Default orientation at final destination
            
        goal_msg.pose.pose.orientation.z = math.sin(angle / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(angle / 2.0)
        
        # Send goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_done_callback)
    
    def navigation_done_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info("Navigation goal accepted")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation completed with result: {result}")
        
        # Move to next mission step
        self.current_mission_step += 1

def main(args=None):
    rclpy.init(args=args)
    
    coordinator = MissionCoordinator()
    
    executor = MultiThreadedExecutor()
    executor.add_node(coordinator)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launching Complete Systems

Complete systems are typically launched using complex launch files:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Include navigation stack
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include manipulation stack
    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_manipulation_pkg'),
                'launch',
                'manipulation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Mission coordinator node
    mission_coordinator = Node(
        package='my_robot_mission',
        executable='mission_coordinator',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': PathJoinSubstitution([
                FindPackageShare('my_robot_description'), 'urdf', 'my_robot.urdf'
            ])}
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        navigation_launch,
        manipulation_launch,
        mission_coordinator
    ])
```

## 5.2 Integration of Perception, Planning, and Control

### Perception-Planning-Control Loop

The perception-planning-control loop is fundamental to autonomous robot operation:

```
Perception → Planning → Control → Robot → Environment → Perception
     ↑                                    ↓
     +------------------------------------+
```

### Perception Pipeline

The perception system processes sensor data to understand the environment:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, LaserScan
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial import distance
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Initialize tools
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, 'pointcloud', self.pointcloud_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, 'camera/color/image_raw', self.image_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        
        # Publishers
        self.objects_pub = self.create_publisher(MarkerArray, 'detected_objects', 10)
        self.transformed_objects_pub = self.create_publisher(
            MarkerArray, 'world_objects', 10
        )
        
        # Object detection parameters
        self.object_min_points = 10
        self.object_max_distance = 5.0
        
    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to numpy array (simplified)
        # In practice, use libraries like sensor_msgs_py or PCL
        try:
            # Extract object clusters from point cloud
            clusters = self.extract_clusters(msg)
            
            # Create markers for detected objects
            markers = MarkerArray()
            for i, cluster in enumerate(clusters):
                marker = self.create_object_marker(cluster, i)
                markers.markers.append(marker)
            
            self.objects_pub.publish(markers)
            
            # Transform object positions to world frame
            world_markers = self.transform_markers_to_world(markers)
            self.transformed_objects_pub.publish(world_markers)
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')
    
    def extract_clusters(self, pointcloud_msg):
        # Simplified clustering algorithm
        # In practice, use PCL or other specialized libraries
        clusters = []
        # ... clustering logic here ...
        return clusters
    
    def create_object_marker(self, cluster, id):
        marker = Marker()
        marker.header.frame_id = pointcloud_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "objects"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Calculate centroid as average of cluster points
        centroid = np.mean(cluster, axis=0)
        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = centroid[2]
        
        # Set size based on cluster size
        marker.scale.x = 0.2  # diameter
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.7  # transparency
        marker.color.r = 1.0  # red
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        return marker
    
    def transform_markers_to_world(self, markers_msg):
        # Transform all markers to world frame
        transformed_markers = MarkerArray()
        
        for marker in markers_msg.markers:
            try:
                # Transform position
                point_stamped = PointStamped()
                point_stamped.header = marker.header
                point_stamped.point.x = marker.pose.position.x
                point_stamped.point.y = marker.pose.position.y
                point_stamped.point.z = marker.pose.position.z
                
                # Transform to world frame (e.g., map frame)
                transformed_point = self.tf_buffer.transform(
                    point_stamped, 'map', timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                # Create new marker in world frame
                world_marker = marker
                world_marker.header.frame_id = 'map'
                world_marker.pose.position = transformed_point.point
                transformed_markers.markers.append(world_marker)
                
            except Exception as e:
                self.get_logger().warn(f'Could not transform marker: {e}')
        
        return transformed_markers
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Perform object detection (using OpenCV or deep learning)
            # This is a simplified example - in practice, use YOLO, SSD, or similar
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Detect red objects (simplified example)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)
            
            lower_red = np.array([170, 100, 100])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)
            
            mask = mask1 + mask2
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Process contours (objects)
            for contour in contours:
                if cv2.contourArea(contour) > 500:  # Filter small objects
                    # Calculate bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # In a real system, you would project this back to 3D
                    # using depth information
                    self.get_logger().info(f'Detected red object at ({x}, {y})')
                    
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def scan_callback(self, msg):
        # Process laser scan data
        # This could include obstacle detection, mapping, etc.
        valid_ranges = [r for r in msg.ranges if not math.isnan(r) and r > 0]
        if valid_ranges:
            min_distance = min(valid_ranges)
            self.get_logger().info(f'Minimum obstacle distance: {min_distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    
    perception_node = PerceptionNode()
    
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    
    perception_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Planning Node

The planning system uses perception data to plan paths and actions:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
import math

class PlanningNode(Node):
    def __init__(self):
        super().__init__('planning_node')
        
        # Action clients
        self.moveit_client = ActionClient(self, MoveGroup, 'move_group')
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10
        )
        self.perception_sub = self.create_subscription(
            MarkerArray, 'world_objects', self.objects_callback, 10
        )
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.initial_pose_callback, 10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.status_pub = self.create_publisher(String, 'planner_status', 10)
        
        # Planning parameters
        self.robot_radius = 0.3  # Robot footprint radius
        self.planning_resolution = 0.1
        self.goal_tolerance = 0.3
        
        # Robot state
        self.current_pose = None
        self.objects = []
        
    def initial_pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def objects_callback(self, msg):
        # Update object positions from perception
        self.objects = []
        for marker in msg.markers:
            obj = {
                'x': marker.pose.position.x,
                'y': marker.pose.position.y,
                'type': marker.ns,
                'id': marker.id
            }
            self.objects.append(obj)
    
    def goal_callback(self, msg):
        if not self.current_pose:
            self.get_logger().warn("Don't know current pose, cannot plan")
            return
            
        # Plan path to goal
        path = self.compute_path(self.current_pose, msg.pose)
        
        if path:
            # Publish the plan
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses = path
            
            self.path_pub.publish(path_msg)
            
            # Publish status update
            status_msg = String()
            status_msg.data = "PATH_PLANNED"
            self.status_pub.publish(status_msg)
            
            self.get_logger().info(f"Published path with {len(path)} waypoints")
        else:
            status_msg = String()
            status_msg.data = "PLANNING_FAILED"
            self.status_pub.publish(status_msg)
            self.get_logger().error("Failed to find a valid path to goal")
    
    def compute_path(self, start_pose, goal_pose):
        # Simplified path planning algorithm
        # In practice, use A*, RRT, or other algorithms
        
        # Check if goal is valid (not inside an object)
        goal_x, goal_y = goal_pose.position.x, goal_pose.position.y
        if self.is_pose_blocked(goal_x, goal_y):
            self.get_logger().error("Goal pose is blocked by an object")
            return None
            
        # Simple direct path calculation (with obstacle avoidance)
        path = []
        
        # Calculate path as a series of waypoints
        start_x = start_pose.position.x
        start_y = start_pose.position.y
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.goal_tolerance:
            # Already at goal
            return [PoseStamped()]
        
        # Break path into segments
        num_waypoints = int(distance / self.planning_resolution)
        for i in range(num_waypoints + 1):
            ratio = i / num_waypoints if num_waypoints > 0 else 0
            x = start_x + ratio * dx
            y = start_y + ratio * dy
            
            # Check for obstacles along path
            if self.is_path_blocked(start_x, start_y, x, y):
                # Try to go around obstacle
                path = self.plan_around_obstacle(start_x, start_y, goal_x, goal_y)
                break
            else:
                # Add waypoint
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                path.append(pose_stamped)
                
                # Update current position for next iteration
                start_x = x
                start_y = y
        
        return path
    
    def is_pose_blocked(self, x, y):
        # Check if pose is too close to any object
        for obj in self.objects:
            dist = math.sqrt((x - obj['x'])**2 + (y - obj['y'])**2)
            if dist < self.robot_radius:
                return True
        return False
    
    def is_path_blocked(self, start_x, start_y, end_x, end_y):
        # Check if the straight line path has obstacles
        # For simplicity, check at intervals along the path
        steps = 20
        for i in range(steps + 1):
            ratio = i / steps
            x = start_x + ratio * (end_x - start_x)
            y = start_y + ratio * (end_y - start_y)
            
            if self.is_pose_blocked(x, y):
                return True
        return False
    
    def plan_around_obstacle(self, start_x, start_y, goal_x, goal_y):
        # Simple obstacle avoidance: try to go around the first blocking object
        # Find the first object blocking the direct path
        for obj in self.objects:
            # Check if object is blocking path
            if self.is_pose_blocked(obj['x'], obj['y']):
                # Plan a path around the object
                # This is a simplified solution - real implementations use more sophisticated algorithms
                path = []
                
                # Go to a point to the left of the object
                left_x = obj['x'] - self.robot_radius * 2
                left_y = obj['y']
                
                # Add path to left of object
                path += self.compute_path_direct(start_x, start_y, left_x, left_y)
                
                # Add path from left of object to goal
                path += self.compute_path_direct(left_x, left_y, goal_x, goal_y)[1:]  # Skip first point to avoid duplication
                
                return path
        
        # If no obstacle found, return direct path
        return self.compute_path_direct(start_x, start_y, goal_x, goal_y)
    
    def compute_path_direct(self, start_x, start_y, end_x, end_y):
        # Compute a direct path without obstacle checking
        path = []
        dx = end_x - start_x
        dy = end_y - start_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        num_waypoints = int(distance / self.planning_resolution)
        for i in range(num_waypoints + 1):
            ratio = i / num_waypoints if num_waypoints > 0 else 0
            x = start_x + ratio * dx
            y = start_y + ratio * dy
            
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path.append(pose_stamped)
        
        return path

def main(args=None):
    rclpy.init(args=args)
    
    planner = PlanningNode()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.3 Performance Considerations

### Performance Metrics

When designing ROS 2 applications, consider these performance metrics:

1. **Real-time Performance**: Ability to meet timing constraints
2. **CPU Utilization**: Efficient use of processing resources
3. **Memory Usage**: Minimize memory footprint
4. **Communication Latency**: Minimize delays in message passing
5. **Bandwidth Usage**: Optimize message sizes and frequency

### Performance Optimization Techniques

#### 1. Efficient Message Handling

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

class OptimizedPerceptionNode(Node):
    def __init__(self):
        super().__init__('optimized_perception')
        
        self.bridge = CvBridge()
        self.image_queue = []
        self.image_mutex = threading.Lock()
        
        # Use small QoS history to minimize memory usage
        qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.optimized_image_callback, qos_profile
        )
        
        # Timer to process images at controlled rate
        self.process_timer = self.create_timer(0.1, self.process_images)
        
    def optimized_image_callback(self, msg):
        # Store image in queue, don't process immediately
        with self.image_mutex:
            if len(self.image_queue) > 2:  # Only keep last 2 images
                self.image_queue.pop(0)
            self.image_queue.append(msg)
    
    def process_images(self):
        # Process images at controlled rate
        with self.image_mutex:
            if not self.image_queue:
                return
                
            latest_image = self.image_queue[-1]  # Use latest image
            self.image_queue.clear()
        
        # Process the image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(latest_image, desired_encoding='bgr8')
            # Perform processing here
            processed_result = self.perform_processing(cv_image)
            
            # Publish results
            # ... 
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def perform_processing(self, cv_image):
        # Your processing logic here
        # Use optimized algorithms, avoid unnecessary copies
        pass

def main(args=None):
    rclpy.init(args=args)
    
    node = OptimizedPerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2. Multi-threading and Callback Groups

```python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import threading

class MultiThreadedController(Node):
    def __init__(self):
        super().__init__('multithreaded_controller')
        
        # Create callback groups
        self.slow_group = MutuallyExclusiveCallbackGroup()
        self.fast_group = MutuallyExclusiveCallbackGroup()
        self.shared_group = ReentrantCallbackGroup()
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Slow processing subscription (e.g., high-level commands)
        self.high_level_sub = self.create_subscription(
            String, 'high_level_command', self.high_level_callback, 
            10, callback_group=self.slow_group
        )
        
        # Fast processing subscription (e.g., safety controller)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.safety_callback, 
            10, callback_group=self.fast_group
        )
        
        # Timer for main control loop
        self.control_timer = self.create_timer(
            0.1, self.control_loop, callback_group=self.shared_group
        )
        
        # State variables
        self.safety_override = False
        self.target_velocity = Twist()
        self.collision_imminent = False
        
        # Threading lock for shared state
        self.state_lock = threading.Lock()
    
    def high_level_callback(self, msg):
        # Process high-level commands
        with self.state_lock:
            if msg.data == "forward":
                self.target_velocity.linear.x = 0.5
            elif msg.data == "stop":
                self.target_velocity.linear.x = 0.0
            elif msg.data == "rotate":
                self.target_velocity.angular.z = 0.5
    
    def safety_callback(self, msg):
        # Process safety-related sensor data (runs at high frequency)
        with self.state_lock:
            # Check for imminent collision
            min_distance = min(msg.ranges) if msg.ranges else float('inf')
            self.collision_imminent = min_distance < 0.5
            
            if self.collision_imminent:
                self.get_logger().warn("Collision imminent! Activating safety override")
                self.safety_override = True
            else:
                self.safety_override = False
    
    def control_loop(self):
        # Main control loop
        with self.state_lock:
            cmd_vel = Twist()
            
            if self.safety_override:
                # Safety override: stop immediately
                # No motion when safety is active
                pass
            else:
                # Normal operation: use target velocity
                cmd_vel = self.target_velocity
        
        self.cmd_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    
    controller = MultiThreadedController()
    
    # Use multi-threaded executor to handle different callback groups
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.4 Best Practices

### 1. Node Design Best Practices

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from std_msgs.msg import Header
import json

class BestPracticeNode(Node):
    def __init__(self):
        super().__init__('best_practice_node')
        
        # 1. Proper parameter declaration with defaults
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('control_frequency', 50)
        self.declare_parameter('debug_mode', False)
        
        # 2. Use appropriate QoS profiles
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.publisher = self.create_publisher(String, 'output_topic', qos_profile)
        
        # 3. Proper error handling and logging
        try:
            self.initialize_components()
        except Exception as e:
            self.get_logger().error(f'Failed to initialize components: {e}')
            raise
        
        # 4. Use timers for periodic tasks
        control_freq = self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(
            1.0/control_freq, self.control_callback
        )
        
        self.get_logger().info('BestPracticeNode initialized successfully')
    
    def initialize_components(self):
        # Initialize node-specific components
        self.velocity_limit = self.get_parameter('max_velocity').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # Validate parameters
        if self.velocity_limit <= 0:
            raise ValueError("max_velocity must be positive")
    
    def control_callback(self):
        # Main control logic
        try:
            # Do work here
            output_msg = String()
            output_msg.data = "Node is running"
            self.publisher.publish(output_msg)
            
            if self.debug_mode:
                self.get_logger().debug("Debug information here")
                
        except Exception as e:
            self.get_logger().error(f'Error in control callback: {e}')
    
    def on_shutdown(self):
        # Cleanup resources
        self.get_logger().info("Shutting down node...")
        # Add cleanup code here

def main(args=None):
    rclpy.init(args=args)
    
    node = BestPracticeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Error Handling and Recovery

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.time import Time
import time
import traceback

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        
        self.publisher = self.create_publisher(String, 'robust_topic', 10)
        
        # State tracking for error recovery
        self.error_count = 0
        self.last_error_time = None
        self.recovery_attempt_count = 0
        self.max_recovery_attempts = 3
        self.error_recovery_timeout = 5.0  # seconds
        
        # Initialize and handle potential errors
        self.components_initialized = False
        self.initialize_components()
        
        # Timer to periodically check system health
        self.health_timer = self.create_timer(1.0, self.check_health)
        
    def initialize_components(self):
        try:
            # Initialize your components here
            self.get_logger().info("Initializing components...")
            
            # Simulate component initialization
            time.sleep(0.1)  # Simulate setup time
            
            self.components_initialized = True
            self.get_logger().info("Components initialized successfully")
            
        except Exception as e:
            self.handle_error("Initialization", e)
            # Attempt recovery
            self.attempt_recovery()
    
    def check_health(self):
        # Periodic health check
        if not self.components_initialized:
            self.get_logger().warn("Components not initialized, attempting reinitialization")
            self.initialize_components()
    
    def safe_publish(self, msg):
        try:
            if self.components_initialized:
                self.publisher.publish(msg)
            else:
                self.get_logger().warn("Cannot publish, components not initialized")
        except Exception as e:
            self.handle_error("Publish", e)
    
    def handle_error(self, context, error):
        self.error_count += 1
        self.last_error_time = self.get_clock().now()
        
        error_msg = f"Error in {context}: {str(error)}"
        error_msg += f"\nTraceback: {traceback.format_exc()}"
        
        self.get_logger().error(error_msg)
        
        # Log specific error details based on error type
        if isinstance(error, ConnectionError):
            self.get_logger().error("Connection error - check network and interfaces")
        elif isinstance(error, ValueError):
            self.get_logger().error("Value error - check parameter values")
        elif isinstance(error, KeyError):
            self.get_logger().error("Key error - check dictionary keys")
        else:
            self.get_logger().error(f"Unknown error type: {type(error)}")
    
    def attempt_recovery(self):
        if self.recovery_attempt_count >= self.max_recovery_attempts:
            self.get_logger().error("Maximum recovery attempts reached, node may be in unrecoverable state")
            return False
        
        self.get_logger().info(f"Attempting recovery, attempt {self.recovery_attempt_count + 1}")
        
        # Reset error state
        self.components_initialized = False
        
        # Wait before retry
        time.sleep(1.0)
        
        # Try to reinitialize
        try:
            self.initialize_components()
            if self.components_initialized:
                self.recovery_attempt_count = 0  # Reset on success
                self.get_logger().info("Recovery successful")
                return True
        except Exception as e:
            self.recovery_attempt_count += 1
            self.get_logger().error(f"Recovery attempt {self.recovery_attempt_count} failed: {e}")
        
        return False

def main(args=None):
    rclpy.init(args=args)
    
    node = RobustNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Unexpected error in main: {e}")
        node.handle_error("Main", e)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

This chapter brought together all the concepts from the previous chapters into comprehensive, applied robotics projects:
- Designing complete ROS 2-based robot applications with proper architecture
- Integrating perception, planning, and control systems in a cohesive system
- Performance considerations and optimization techniques
- Best practices for robust and maintainable robot software

Creating complete robotics applications requires combining all the concepts learned in this module - from basic ROS 2 fundamentals to simulation integration. The key is to design modular, well-architected systems that can be tested and validated effectively.

## Next Steps

You've now completed all chapters of Module 1: The Robotic Nervous System (ROS 2). This module provided a comprehensive introduction to ROS 2 middleware for humanoid robot control, covering nodes, topics, services, actions, Python integration with rclpy, and URDF. Continue to [Module 2: The Digital Twin (Gazebo & Unity)](../../002-digital-twin-simulation/001-module-overview.md) to learn about simulation technologies for robotics.