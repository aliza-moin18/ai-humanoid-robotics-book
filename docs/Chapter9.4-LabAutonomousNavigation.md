# Chapter 9.4: Lab: Autonomous Navigation

## Objective

Implement a complete autonomous navigation system using Nav2 and Isaac tools that demonstrates path planning, obstacle avoidance, and goal-directed navigation in both simulated and real-world scenarios.

## Materials Required

### Software
- Isaac Sim 2023.1.1
- Isaac ROS packages (navigation-related)
- ROS 2 Humble Hawksbill
- Navigation2 (Nav2) packages
- CUDA 12.x with appropriate GPU drivers
- Python 3.8-3.10

### Simulation Assets
- Humanoid robot model with differential drive or similar mobility
- LiDAR and camera sensors configured on the robot
- Sample environments for navigation testing
- Navigation configuration files

## Setup Instructions

### Step 1: Environment Verification
1. Verify Isaac Sim installation:
   ```bash
   cd /opt/nvidia/isaac_sim
   ./python.sh -c "import omni; print('Isaac Sim accessible')"
   ```

2. Verify Isaac ROS and Nav2 packages:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 pkg list | grep -E "(nav2|isaac)"
   ```

3. Check GPU access:
   ```bash
   python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
   ```

4. Ensure Nav2 is installed:
   ```bash
   ros2 pkg list | grep nav2
   ```

### Step 2: Create Navigation Workspace
1. Create a workspace for navigation configuration:
   ```bash
   mkdir -p ~/isaac_nav_ws/src/isaac_nav2_bringup
   cd ~/isaac_nav_ws/src/isaac_nav2_bringup
   mkdir -p config launch maps rviz
   ```

2. Create navigation parameters file:
   ```bash
   # ~/isaac_nav_ws/src/isaac_nav2_bringup/config/nav2_params.yaml
   amcl:
     ros__parameters:
       use_sim_time: True
       alpha1: 0.2
       alpha2: 0.2
       alpha3: 0.2
       alpha4: 0.2
       alpha5: 0.2
       base_frame_id: "base_link"
       beam_skip_distance: 0.5
       beam_skip_error_threshold: 0.9
       beam_skip_threshold: 0.3
       do_beamskip: false
       global_frame_id: "map"
       lambda_short: 0.1
       likelihood_max_dist: 2.0
       set_initial_pose: true
       initial_pose:
         x: 0.0
         y: 0.0
         z: 0.0
         yaw: 0.0
       tf_broadcast: true
       transform_tolerance: 1.0
       update_min_a: 0.2
       update_min_d: 0.25
       z_hit: 0.5
       z_max: 0.05
       z_rand: 0.5
       z_short: 0.05
       scan_topic: scan

   bt_navigator:
     ros__parameters:
       use_sim_time: True
       global_frame: map
       robot_base_frame: base_link
       odom_topic: /odom
       bt_loop_duration: 10
       default_server_timeout: 20
       # Specify the path where the XML file is located
       behavior_tree_xml_filename: "navigate_w_replanning_and_recovery.xml"
       plugin_lib_names:
       - nav2_compute_path_to_pose_action_bt_node
       - nav2_compute_path_through_poses_action_bt_node
       - nav2_follow_path_action_bt_node
       - nav2_back_up_action_bt_node
       - nav2_spin_action_bt_node
       - nav2_wait_action_bt_node
       - nav2_clear_costmap_service_bt_node
       - nav2_is_stuck_condition_bt_node
       - nav2_goal_reached_condition_bt_node
       - nav2_goal_updated_condition_bt_node
       - nav2_initial_pose_received_condition_bt_node
       - nav2_reinitialize_global_localization_service_bt_node
       - nav2_rate_controller_bt_node
       - nav2_distance_controller_bt_node
       - nav2_speed_controller_bt_node
       - nav2_truncate_path_action_bt_node
       - nav2_truncate_path_local_action_bt_node
       - nav2_goal_updater_node_bt_node
       - nav2_recovery_node_bt_node
       - nav2_pipeline_sequence_bt_node
       - nav2_round_robin_node_bt_node
       - nav2_transform_available_condition_bt_node
       - nav2_time_expired_condition_bt_node
       - nav2_path_expiring_timer_condition
       - nav2_distance_traveled_condition_bt_node
       - nav2_single_trigger_bt_node
       - nav2_is_battery_low_condition_bt_node
       - nav2_navigate_through_poses_action_bt_node
       - nav2_navigate_to_pose_action_bt_node
       - nav2_remove_passed_goals_action_bt_node
       - nav2_planner_selector_bt_node
       - nav2_controller_selector_bt_node
       - nav2_goal_checker_selector_bt_node
       - nav2_controller_cancel_bt_node
       - nav2_path_longer_on_approach_bt_node
       - nav2_wait_at_waypoint_bt_node
       - nav2_spin_frame_aligner_bt_node

   controller_server:
     ros__parameters:
       use_sim_time: True
       controller_frequency: 20.0
       min_x_velocity_threshold: 0.001
       min_y_velocity_threshold: 0.5
       min_theta_velocity_threshold: 0.001
       progress_checker_plugin: "progress_checker"
       goal_checker_plugin: "goal_checker"
       controller_plugins: ["FollowPath"]
       
       # DWB Controller
       FollowPath:
         plugin: "dwb_core::DWBLocalPlanner"
         debug_trajectory_details: True
         min_vel_x: 0.0
         min_vel_y: 0.0
         max_vel_x: 0.5
         max_vel_y: 0.0
         max_vel_theta: 1.0
         min_speed_xy: 0.0
         max_speed_xy: 0.5
         min_speed_theta: 0.0
         acc_lim_x: 2.5
         acc_lim_y: 0.0
         acc_lim_theta: 3.2
         decel_lim_x: -2.5
         decel_lim_y: 0.0
         decel_lim_theta: -3.2
         vx_samples: 20
         vy_samples: 0
         vtheta_samples: 40
         sim_time: 1.7
         linear_granularity: 0.05
         angular_granularity: 0.1
         transform_tolerance: 0.2
         xy_goal_tolerance: 0.25
         yaw_goal_tolerance: 0.05
         stateful: True
         oscillation_reset_dist: 0.05
         publisher_frequency: 0.0
         goal_checker: "goal_checker"

   local_costmap:
     local_costmap:
       ros__parameters:
         update_frequency: 5.0
         publish_frequency: 2.0
         global_frame: odom
         robot_base_frame: base_link
         use_sim_time: True
         rolling_window: true
         width: 6
         height: 6
         resolution: 0.05
         robot_radius: 0.22
         plugins: ["obstacle_layer", "inflation_layer"]
         inflation_layer:
           plugin: "nav2_costmap_2d::InflationLayer"
           cost_scaling_factor: 3.0
           inflation_radius: 0.55
         obstacle_layer:
           plugin: "nav2_costmap_2d::ObstacleLayer"
           enabled: True
           observation_sources: scan
           scan:
             topic: /scan
             max_obstacle_height: 2.0
             clearing: True
             marking: True
             data_type: "LaserScan"
             raytrace_range: 6.0
             obstacle_range: 5.0
             origin_z: 0.0
             z_offset: 0.0
             z_scale: 1.0
             inf_is_valid: True
         always_send_full_costmap: True

   global_costmap:
     global_costmap:
       ros__parameters:
         update_frequency: 1.0
         publish_frequency: 1.0
         global_frame: map
         robot_base_frame: base_link
         use_sim_time: True
         robot_radius: 0.22
         resolution: 0.05
         track_unknown_space: false
         plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
         obstacle_layer:
           plugin: "nav2_costmap_2d::ObstacleLayer"
           enabled: true
           observation_sources: scan
           scan:
             topic: /scan
             max_obstacle_height: 2.0
             clearing: true
             marking: true
             data_type: "LaserScan"
         static_layer:
           plugin: "nav2_costmap_2d::StaticLayer"
           map_subscribe_transient_local: true
         inflation_layer:
           plugin: "nav2_costmap_2d::InflationLayer"
           cost_scaling_factor: 3.0
           inflation_radius: 0.55

   planner_server:
     ros__parameters:
       expected_planner_frequency: 20.0
       planner_plugins: ["GridBased"]
       GridBased:
         plugin: "nav2_navfn_planner::NavfnPlanner"
         tolerance: 0.5
         use_astar: false
         allow_unknown: true

   recovery_behaviors_server:
     ros__parameters:
       costmap_topic: local_costmap/costmap_raw
       footprint_topic: local_costmap/published_footprint
       cycle_frequency: 10.0
       recovery_plugins: ["spin", "backup", "wait"]
       recovery_plugin_types: ["nav2_recoveries::Spin", "nav2_recoveries::BackUp", "nav2_recoveries::Wait"]
       spin:
         plugin: "nav2_recoveries::Spin"
         sim_frequency: 20.0
         cycle_frequency: 5.0
         acceleration: 1.0
         max_rotational_vel: 1.0
         min_rotational_vel: 0.4
         rotational_acc_lim: 3.2
   ```

### Step 3: Launch Isaac Sim with Navigation Setup
1. Launch Isaac Sim with a robot model:
   ```bash
   cd /opt/nvidia/isaac_sim
   ./isaac-sim.sh --exec "omni.isaac.examples.simple_robots.carter_franka_pick_place"
   ```

2. Ensure the robot has appropriate sensors configured (LiDAR, camera, IMU).

3. Enable ROS Bridge in Isaac Sim to publish sensor data.

## Procedures

### Procedure 1: Initialize Navigation System
1. Create a launch file for the navigation system:
   ```bash
   # ~/isaac_nav_ws/src/isaac_nav2_bringup/launch/navigation.launch.py
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
   from launch_ros.actions import Node
   from launch_ros.substitutions import FindPackageShare
   
   def generate_launch_description():
       # Declare launch arguments
       use_sim_time = LaunchConfiguration('use_sim_time', default='True')
       params_file = LaunchConfiguration('params_file', default='nav2_params.yaml')
       
       # Navigation launch from nav2_bringup
       nav2_bringup_launch = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               FindPackageShare('nav2_bringup'), '/launch', '/navigation_launch.py'
           ]),
           launch_arguments={
               'use_sim_time': use_sim_time,
               'params_file': params_file
           }.items()
       )
       
       # RViz node for visualization
       rviz_config_file = PathJoinSubstitution([
           FindPackageShare('isaac_nav2_bringup'),
           'rviz',
           'navigation_config.rviz'
       ])
       
       rviz_node = Node(
           package='rviz2',
           executable='rviz2',
           name='rviz2',
           arguments=['-d', rviz_config_file],
           parameters=[{'use_sim_time': use_sim_time}],
           output='screen'
       )
       
       return LaunchDescription([
           nav2_bringup_launch,
           rviz_node
       ])
   ```

2. Create an RViz configuration file:
   ```yaml
   # ~/isaac_nav_ws/src/isaac_nav2_bringup/rviz/navigation_config.rviz
   Panels:
     - Class: rviz_common/Displays
       Help Height: 78
       Name: Displays
       Property Tree Widget:
         Expanded:
           - /Global Options1
           - /Status1
           - /Map1
           - /Local Costmap1
           - /Global Costmap1
           - /Local Plan1
           - /Global Plan1
           - /Robot1
           - /Goals1
         Splitter Ratio: 0.5
       Tree Height: 777
     - Class: rviz_common/Selection
       Name: Selection
     - Class: rviz_common/Tool Properties
       Expanded:
         - /2D Goal1
         - /Publish Point1
       Name: Tool Properties
       Splitter Ratio: 0.5886790156364441
     - Class: rviz_common/Views
       Expanded:
         - /Current View1
       Name: Views
       Splitter Ratio: 0.5
     - Class: nav2_rviz_plugins/Navigation 2
       Name: Navigation 2
   Preferences:
     Prompt Save on Exit: true
   Toolbars:
     tool_button_style: 2
   Visualization Manager:
     Class: ""
     Displays:
       - Alpha: 0.5
         Cell Size: 1
         Class: rviz_default_plugins/Grid
         Color: 160; 160; 164
         Enabled: true
         Line Style:
           Line Width: 0.029999999329447746
           Value: Lines
         Name: Grid
         Normal Cell Count: 0
         Offset:
           X: 0
           Y: 0
           Z: 0
         Plane: XY
         Plane Cell Count: 100
         Reference Frame: <Fixed Frame>
         Value: true
       - Alpha: 1
         Class: rviz_default_plugins/RobotModel
         Collision Enabled: false
         Enabled: true
         Links:
           All Links Enabled: true
           Expand Joint Details: false
           Expand Link Details: false
           Expand Tree: false
           Link Tree Style: ""
         Name: Robot
         Robot Description: robot_description
         TF Prefix: ""
         Update Interval: 0
         Value: true
         Visual Enabled: true
       - Class: rviz_default_plugins/TF
         Enabled: true
         Frame Timeout: 15
         Frames:
           All Enabled: true
         Marker Scale: 1
         Name: TF
         Show Arrows: true
         Show Axes: true
         Show Names: false
         Tree:
           {}
         Update Interval: 0
         Value: true
       - Alpha: 0.699999988079071
         Class: rviz_default_plugins/Map
         Color Scheme: map
         Draw Behind: false
         Enabled: true
         Name: Map
         Topic:
           Depth: 5
           Durability Policy: Transient Local
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /map
         Update Topic:
           Depth: 5
           Durability Policy: Transient Local
           History Policy: Keep Last
           Reliability Policy: Reliable
         Use Timestamp: false
         Value: true
       - Alpha: 1
         Buffer Length: 1
         Class: rviz_default_plugins/Path
         Color: 255; 0; 0
         Enabled: true
         Head Diameter: 0.30000001192092896
         Head Length: 0.20000000298023224
         Length: 0.30000001192092896
         Line Style: Lines
         Line Width: 0.029999999329447746
         Name: Global Plan
         Offset:
           X: 0
           Y: 0
           Z: 0
         Pose Color: 255; 85; 255
         Pose Style: Arrows
         Radius: 0.029999999329447746
         Shaft Diameter: 0.10000000149011612
         Shaft Length: 0.10000000149011612
         Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /plan
         Value: true
       - Alpha: 1
         Buffer Length: 1
         Class: rviz_default_plugins/Path
         Color: 25; 255; 0
         Enabled: true
         Head Diameter: 0.30000001192092896
         Head Length: 0.20000000298023224
         Length: 0.30000001192092896
         Line Style: Lines
         Line Width: 0.029999999329447746
         Name: Local Plan
         Offset:
           X: 0
           Y: 0
           Z: 0
         Pose Color: 255; 85; 255
         Pose Style: Arrows
         Radius: 0.029999999329447746
         Shaft Diameter: 0.10000000149011612
         Shaft Length: 0.10000000149011612
         Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /local_plan
         Value: true
       - Alpha: 0.699999988079071
         Class: rviz_default_plugins/Map
         Color Scheme: costmap
         Draw Behind: false
         Enabled: true
         Name: Local Costmap
         Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /local_costmap/costmap
         Update Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
         Use Timestamp: false
         Value: true
       - Alpha: 0.699999988079071
         Class: rviz_default_plugins/Map
         Color Scheme: costmap
         Draw Behind: false
         Enabled: true
         Name: Global Costmap
         Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /global_costmap/costmap
         Update Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
         Use Timestamp: false
         Value: true
       - Alpha: 1
         Class: rviz_default_plugins/Polygon
         Color: 25; 255; 0
         Enabled: true
         Name: Robot Footprint
         Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /local_costmap/published_footprint
         Value: true
       - Alpha: 1
         Class: rviz_default_plugins/PointStamped
         Color: 204; 41; 204
         Enabled: true
         History Length: 1
         Name: Goal Pose
         Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /goal_pose
         Value: true
     Enabled: true
     Global Options:
       Background Color: 48; 48; 48
       Fixed Frame: map
       Frame Rate: 30
     Name: root
     Tools:
       - Class: rviz_default_plugins/Interact
         Hide Inactive Objects: true
       - Class: rviz_default_plugins/SetGoal
         Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /goal_pose
       - Class: rviz_default_plugins/SetInitialPose
         Topic:
           Depth: 5
           Durability Policy: Volatile
           History Policy: Keep Last
           Reliability Policy: Reliable
           Value: /initialpose
     Transformation:
       Current:
         Class: rviz_default_plugins/TF
     Value: true
     Views:
       Current:
         Angle: -1.5700000524520874
         Class: rviz_default_plugins/TopDownOrtho
         Enable Stereo Rendering:
           Stereo Eye Separation: 0.05999999865889549
           Stereo Focal Distance: 1
           Swap Stereo Eyes: false
           Value: false
         Invert Z Axis: false
         Name: Current View
         Near Clip Distance: 0.009999999776482582
         Scale: 100
         Target Frame: <Fixed Frame>
         Value: TopDownOrtho (rviz_default_plugins)
         X: 0
         Y: 0
       Saved: ~
   Window Geometry:
     Displays:
       collapsed: false
     Height: 1043
     Hide Left Dock: false
     Hide Right Dock: true
     Navigation 2:
       collapsed: false
     QMainWindow State: 000000ff00000000fd00000004000000000000015600000391fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d00000391000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f00000391fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d00000391000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d00650100000000000004500000000000000000000003a10000039100000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
     Width: 1855
     X: 65
     Y: 24
   ```

3. Launch the navigation system:
   ```bash
   cd ~/isaac_nav_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch isaac_nav2_bringup navigation.launch.py
   ```

### Procedure 2: Configure Isaac Sim for Navigation
1. In Isaac Sim, ensure ROS Bridge is enabled and publishing:
   - Camera data to `/camera/image_rect_color`
   - LiDAR data to `/scan` or `/laser_scan`
   - IMU data to `/imu`
   - Odometry data to `/odom`
   - TF transforms between all relevant frames

2. Set Isaac Sim to use simulation time:
   ```bash
   export GAZEBO_USE_SIM_TIME=true  # If using Gazebo-style simulation
   # For Isaac Sim, ensure Isaac Sim is publishing simulation time messages
   ```

3. Verify sensor data is flowing:
   ```bash
   # Check available topics
   ros2 topic list
   
   # Monitor sensor data
   ros2 topic echo /scan --field ranges | head -n 5
   ros2 topic echo /camera/image_rect_color --field header.seq | head -n 5
   ```

### Procedure 3: Test Navigation in Isaac Sim
1. **Send a simple navigation goal in RViz**:
   - In RViz, click the "2D Goal Pose" button
   - Click on the map to set a goal position
   - Verify the robot plans a path and navigates to the goal

2. **Monitor navigation progress**:
   ```bash
   # Check the navigation action status
   ros2 action list
   ros2 action info /navigate_to_pose
   
   # Monitor costmaps
   ros2 topic echo /local_costmap/costmap_updates --field header.seq | head -n 5
   ros2 topic echo /global_costmap/costmap_updates --field header.seq | head -n 5
   
   # Check robot position
   ros2 topic echo /amcl_pose --field pose.pose.position | head -n 5
   ```

3. **Test obstacle avoidance**:
   - Add obstacles in Isaac Sim to the navigation path
   - Verify the robot detects obstacles and replans
   - Check that the robot maintains safe distances from obstacles

4. **Test recovery behaviors**:
   - Place the robot in a narrow position where it might get stuck
   - Trigger recovery behaviors by forcing the robot into difficult positions
   - Verify spin, backup, and wait recovery behaviors work

### Procedure 4: Advanced Navigation Tasks
1. **Navigation with Isaac ROS Perception**:
   - Integrate Isaac ROS perception nodes into the navigation pipeline
   - Use Isaac ROS for enhanced obstacle detection
   - Test navigation with multiple sensor inputs

2. **Multi-goal Navigation**:
   - Create a task with multiple waypoints
   - Implement a simple task management system
   - Test transitions between different goals

3. **Dynamic Obstacle Handling**:
   - Add moving obstacles in Isaac Sim
   - Test predictive obstacle avoidance
   - Monitor navigation performance with dynamic obstacles

### Procedure 5: Performance Evaluation
1. **Collect Performance Metrics**:
   - Path efficiency (actual vs. optimal path length)
   - Time to reach goal
   - Number of replanning events
   - Collision avoidance effectiveness

2. **Run Navigation Tests**:
   ```bash
   # Create a simple navigation test script
   cat > ~/isaac_nav_ws/src/isaac_nav2_bringup/test/navigation_test.py << EOF
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import time
import math


class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.test_results = []
        
    def send_goal(self, x, y, theta=0.0):
        # Wait for the action server to be available
        self.action_client.wait_for_server()
        
        # Create the goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Send the goal
        self.get_logger().info(f'Sending goal to position: ({x}, {y})')
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
            
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.test_results.append(result)
        

def main():
    rclpy.init()
    
    tester = NavigationTester()
    
    # Define a series of test goals
    test_goals = [
        (2.0, 0.0),   # Simple forward
        (2.0, 2.0),   # Diagonal
        (-1.0, 2.0),  # Backwards and sideways
        (0.0, 0.0)    # Return to start
    ]
    
    # Send each goal sequentially
    for x, y in test_goals:
        tester.send_goal(x, y)
        time.sleep(5)  # Wait before sending next goal
    
    # Run for some time to process goals
    rclpy.spin_once(tester, timeout_sec=20)
    
    # Print results
    print("Navigation test results:")
    for i, result in enumerate(tester.test_results):
        print(f"Test {i+1}: Success={result.result}")
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

   # Make the script executable
   chmod +x ~/isaac_nav_ws/src/isaac_nav2_bringup/test/navigation_test.py
   ```

2. **Run the navigation test**:
   ```bash
   source ~/isaac_nav_ws/install/setup.bash
   python3 ~/isaac_nav_ws/src/isaac_nav2_bringup/test/navigation_test.py
   ```

## Assessment Criteria

### Completion Requirements
- [ ] Successfully launch Nav2 navigation system with Isaac tools
- [ ] Configure costmaps and local/global planners appropriately
- [ ] Demonstrate path planning to user-specified goals
- [ ] Show effective obstacle detection and avoidance
- [ ] Execute successful navigation in Isaac Sim environment
- [ ] Validate navigation performance metrics
- [ ] Complete advanced navigation tasks (multi-goal, dynamic obstacles)

### Performance Metrics
- [ ] Robot successfully reaches at least 80% of navigation goals
- [ ] Average path efficiency > 85% of optimal straight-line distance
- [ ] Navigation completion time < 2 minutes per 10m distance
- [ ] Zero collisions during navigation tests
- [ ] Recovery behaviors activate appropriately when needed
- [ ] System maintains stable operation throughout testing

### Verification Steps
1. Test basic navigation:
   ```bash
   # In one terminal, check if navigation is working
   ros2 topic echo /initialpose  # Set initial pose in RViz
   # Then set goal pose and verify navigation
   ```

2. Verify sensor integration:
   ```bash
   # Check if all required sensor topics are publishing
   ros2 topic echo /scan --field ranges | head -n 1
   ros2 topic echo /odom --field pose.pose.position | head -n 1
   ```

3. Validate costmap functionality:
   ```bash
   # Check if costmaps update with obstacles
   ros2 run nav2_msgs/costmap_display
   ```

## Lab Report Requirements

Students must submit a lab report documenting:

1. **System Configuration**: Detailed account of the navigation system setup, including Nav2 configuration and Isaac Sim settings.

2. **Navigation Performance**: Results of navigation tests, including path efficiency, success rates, and time metrics.

3. **Obstacle Handling**: Documentation of how the robot handled various obstacle scenarios.

4. **Challenges Encountered**: Any issues faced during the lab and how they were resolved.

5. **Parameter Tuning**: Description of any Nav2 parameters that were adjusted and the rationale.

6. **Integration Quality**: Assessment of how well Nav2 integrates with Isaac tools.

7. **Performance Analysis**: Analysis of the navigation system's performance under different conditions.

## Extensions

### Advanced Navigation Tasks
For advanced students:
1. Implement a custom path planner plugin for Nav2
2. Integrate Isaac ROS perception for enhanced navigation
3. Implement learning-based navigation approaches
4. Create navigation tests with complex dynamic environments

### Performance Optimization
1. Optimize navigation parameters for specific robot dynamics
2. Implement hierarchical navigation approaches
3. Add semantic navigation capabilities using Isaac perception
4. Create adaptive navigation strategies based on environment types

### Research Extensions
1. Investigate sim-to-real transfer of navigation capabilities
2. Implement multi-robot navigation coordination
3. Add navigation planning with energy efficiency considerations
4. Create navigation systems with human-aware behavior

## Troubleshooting Guide

### Common Issues and Solutions

1. **No Path Planning**:
   - Check if costmaps are updating
   - Verify robot pose estimates
   - Ensure Nav2 lifecycle nodes are active
   - Validate map and localization

2. **Collision with Obstacles**:
   - Check costmap inflation settings
   - Verify obstacle detection and sensor configuration
   - Adjust controller parameters for better obstacle avoidance
   - Check robot footprint settings

3. **Controller Issues**:
   - Check if local planner is computing trajectories
   - Verify robot kinematics constraints
   - Adjust velocity limits and acceleration limits
   - Tune trajectory evaluation parameters

4. **Localization Problems**:
   - Verify AMCL is running and publishing pose
   - Check if the map is loaded correctly
   - Validate sensor configuration for localization
   - Ensure proper TF tree between frames

5. **Performance Issues**:
   - Monitor CPU and GPU usage
   - Adjust update frequencies appropriately
   - Simplify costmap resolution if needed
   - Check for bottlenecks in sensor processing

6. **Isaac Sim Integration Issues**:
   - Verify ROS Bridge is publishing required topics
   - Check simulation time synchronization
   - Ensure proper coordinate frame relationships
   - Validate sensor configuration in Isaac Sim