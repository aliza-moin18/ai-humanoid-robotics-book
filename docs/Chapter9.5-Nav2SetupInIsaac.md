# Chapter 9.5: Nav2 Setup Procedures in Isaac Environment

## Learning Objectives
- Install and configure Nav2 for use with Isaac Sim and Isaac ROS
- Set up the necessary ROS 2 workspace and dependencies
- Configure Nav2 parameters specifically for Isaac-based robots
- Validate the Nav2-Isaac integration
- Troubleshoot common setup issues

## Estimated Completion Time: 2 hours

## Prerequisites
- Isaac Sim installed and verified (covered in Chapter 7.4)
- Isaac ROS packages installed and configured
- ROS 2 Humble Hawksbill installed
- NVIDIA GPU with appropriate drivers
- Basic understanding of ROS 2 workspace setup

## Overview of Nav2-Isaac Integration Setup

Setting up Navigation2 (Nav2) with Isaac tools involves several critical steps to ensure proper integration between the navigation stack and Isaac's simulation and perception capabilities. This setup process includes:

1. Installing Nav2 packages and dependencies
2. Creating a dedicated workspace for Nav2-Isaac integration
3. Configuring Nav2 parameters for Isaac environments
4. Setting up launch files to coordinate Nav2 and Isaac Sim
5. Validating the integration through basic navigation tests

## System Requirements

### Hardware Requirements
- NVIDIA RTX 3080 or higher (RTX 4090 recommended)
- Ubuntu 22.04 LTS
- 32GB RAM minimum (64GB recommended)
- 1TB SSD storage for simulation assets
- Compatible humanoid robot model

### Software Requirements
- NVIDIA GPU drivers (latest version)
- CUDA 12.x
- Isaac Sim 2023.1.1
- Isaac ROS 3.1
- ROS 2 Humble Hawksbill
- Navigation2 (Nav2) packages
- Python 3.8-3.10

## Step 1: Install Nav2 Packages

### Install Nav2 via APT (Recommended)
```bash
# Update package lists
sudo apt update

# Install Nav2 packages
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-gui

# Install additional dependencies
sudo apt install ros-humble-dwb-core ros-humble-dwb-plugins
sudo apt install ros-humble-nav2-rviz-plugins
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-interactive-markers
```

### Verify Installation
```bash
# Check if Nav2 packages are installed
ros2 pkg list | grep nav2

# You should see packages like:
# nav2_amcl
# nav2_behavior_tree
# nav2_bt_navigator
# nav2_controller
# nav2_core
# nav2_costmap_2d
# nav2_dwb_controller
# nav2_lifecycle_manager
# nav2_map_server
# nav2_msgs
# nav2_planner
# nav2_recoveries
# nav2_rotation_shifter
# nav2_server
# nav2_system_tests
# nav2_util
# nav2_velocity_smoother
# nav2_voxel_grid
# nav2_waypoint_follower
```

## Step 2: Create Nav2-Isaac Workspace

### Create the Workspace
```bash
# Create a dedicated workspace for Nav2-Isaac integration
mkdir -p ~/isaac_nav2_ws/src
cd ~/isaac_nav2_ws

# Create necessary directories for configuration
mkdir -p src/isaac_nav2_bringup/config
mkdir -p src/isaac_nav2_bringup/launch
mkdir -p src/isaac_nav2_bringup/rviz
mkdir -p src/isaac_nav2_bringup/maps
```

### Create Package.xml for isaac_nav2_bringup
```xml
<!-- ~/isaac_nav2_ws/src/isaac_nav2_bringup/package.xml -->
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>isaac_nav2_bringup</name>
  <version>1.0.0</version>
  <description>Nav2 configuration and launch files for Isaac-based robots</description>
  <maintainer email="student@university.edu">Student</maintainer>
  <license>Apache-2.0</license>

  <depend>nav2_bringup</depend>
  <depend>nav2_map_server</depend>
  <depend>nav2_planner</depend>
  <depend>nav2_controller</depend>
  <depend>nav2_behaviors</depend>
  <depend>nav2_bt_navigator</depend>
  <depend>nav2_util</depend>
  <depend>nav2_lifecycle_manager</depend>
  <depend>isaac_ros_common</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Create CMakeLists.txt for isaac_nav2_bringup
```cmake
# ~/isaac_nav2_ws/src/isaac_nav2_bringup/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(isaac_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Step 3: Configure Nav2 Parameters for Isaac

### Create Isaac-Specific Nav2 Parameters
```yaml
# ~/isaac_nav2_ws/src/isaac_nav2_bringup/config/nav2_isaac_params.yaml
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

    # DWB Controller for Isaac-based robots
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

velocity_smoother:
  ros__parameters:
    use_sim_time: True
    smoothing_frequency: 20.0
    scale_velocities: false
    velocity_threshold: 0.0
    velocity_scale_tolerance: 0.0
    max_velocity: [1.0, 1.0, 1.0]
    min_velocity: [-1.0, -1.0, -1.0]
    velocity_scaling_type: 0
    feedback: 0
    velocity_tracker_node:
      use_sim_time: True

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

## Step 4: Create Isaac-Specific Launch Files

### Navigation Launch File
```python
# ~/isaac_nav2_ws/src/isaac_nav2_bringup/launch/isaac_navigation_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    params_file = LaunchConfiguration('params_file', default='nav2_isaac_params.yaml')
    autostart = LaunchConfiguration('autostart', default='True')
    use_composition = LaunchConfiguration('use_composition', default='False')
    container_name = LaunchConfiguration('container_name', default='nav2_container')
    use_respawn = LaunchConfiguration('use_respawn', default='False')
    log_level = LaunchConfiguration('log_level', default='info')
    rviz_config_file = LaunchConfiguration('rviz_config', default='isaac_navigation.rviz')

    # Create parameter file path
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
    }

    # Create configuration file path
    isaac_param_file = PathJoinSubstitution([
        FindPackageShare('isaac_nav2_bringup'),
        'config',
        params_file
    ])

    configured_params = RewrittenYaml(
        source_file=isaac_param_file,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Launch the main Nav2 nodes
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'), '/launch', '/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'use_composition': use_composition,
            'container_name': container_name,
            'use_respawn': use_respawn,
            'log_level': log_level
        }.items()
    )

    # Launch RViz with Isaac-specific configuration
    rviz_config = PathJoinSubstitution([
        FindPackageShare('isaac_nav2_bringup'),
        'rviz',
        rviz_config_file
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Isaac Perception Integration Node
    perception_integration_node = Node(
        package='isaac_nav2_bringup',
        executable='perception_integration_node',
        name='perception_integration_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0
    )

    return LaunchDescription([
        SetParameter('use_sim_time', use_sim_time),
        nav2_bringup_launch,
        rviz_node,
        perception_integration_node
    ])
```

### RViz Configuration for Isaac Navigation
```yaml
# ~/isaac_nav2_ws/src/isaac_nav2_bringup/rviz/isaac_navigation.rviz
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
        - /Isaac Sensors1
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
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Isaac Sensors
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /isaac_ros/laser_scan_visualization
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

## Step 5: Build the Workspace

### Source ROS 2 and Isaac ROS
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Source Isaac ROS workspace
source ~/isaac_ros_ws/install/setup.bash
```

### Build the Isaac Nav2 Workspace
```bash
cd ~/isaac_nav2_ws
colcon build --packages-select isaac_nav2_bringup
source install/setup.bash
```

## Step 6: Verify the Setup

### Check Package Installation
```bash
# Verify the package was built correctly
ros2 pkg list | grep isaac_nav2
# Should output: isaac_nav2_bringup

# Check launch files
ros2 launch isaac_nav2_bringup isaac_navigation_launch.py --show-all
```

### Test Launch
```bash
# Try launching just the configuration to verify syntax
# Note: This won't run properly without Isaac Sim running
ros2 launch isaac_nav2_bringup isaac_navigation_launch.py
```

## Step 7: Isaac Sim Integration Setup

### Configure Isaac Sim for Navigation
1. Launch Isaac Sim with a robot that has navigation capabilities
2. Ensure the robot has appropriate sensors (LiDAR, camera, IMU)
3. Enable the ROS Bridge in Isaac Sim
4. Configure the robot to publish required navigation topics:
   - `/scan` - LiDAR data
   - `/odom` - Odometry data
   - `/tf` and `/tf_static` - Transform data
   - `/camera/image_rect_color` - Camera data (if using visual navigation)

### Isaac Sim Navigation Configuration
```python
# Example Isaac Sim script to configure navigation
import omni
from omni.isaac.kit import SimulationApp
import carb
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils import stage, geometry, transforms
from omni.isaac.range_sensor import _range_sensor
from omni.isaac.core.utils.prims import get_prim_at_path
import omni.isaac.core.utils.numpy as np_utils

# Initialize simulation application
config = {
    "headless": False,
    "rendering": True,
    "simulation": True,
    "enable_cameras": True,
    "enable_lidar": True
}

simulation_app = SimulationApp(config)

# Import ROS bridge
from omni.isaac.ros_bridge import get_ros_bridge_instance

# Create the world
world = World(stage_units_in_meters=1.0)

# Create ground plane
geometry.create_ground_plane("/World/GroundPlane", "x", 10.0, 0.0, 0.5, False)

# Create a simple robot (replace with your actual robot)
robot = world.scene.add(
    geometry.Cuboid(
        prim_path="/World/Robot",
        name="robot",
        position=[0.0, 0.0, 0.2],
        size=0.4,
        color=np.array([0.1, 0.1, 0.9])
    )
)

# Create LiDAR sensor on the robot
lidar = _range_sensor.acquire_lidar_sensor_interface()
lidar_sensor = world.scene.add(
    geometry.RaycastCuboid(
        prim_path="/World/Robot/Lidar",
        name="lidar_sensor",
        position=[0.0, 0.0, 0.3],
        size=0.1
    )
)

# Enable ROS bridge
ros_bridge = get_ros_bridge_instance()
ros_bridge.ros_publish("/World/Robot/Lidar", "LaserScan", "/scan")
ros_bridge.ros_publish("/World/Robot", "Odometry", "/odom")
ros_bridge.ros_subscribe("/World/Robot", "Twist", "/cmd_vel")

# Reset and step the world
world.reset()

# Run simulation for a few steps to ensure everything is set up
for i in range(10):
    world.step(render=True)

print("Isaac Sim navigation setup complete")

# Close the simulation
world.clear()
simulation_app.close()
```

## Troubleshooting Common Issues

### Issue 1: Package Not Found
**Problem**: `ros2 pkg list` doesn't show Nav2 packages
**Solution**: 
```bash
# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash
# Reinstall if needed
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### Issue 2: Parameter File Not Loading
**Problem**: Nav2 nodes fail to start due to parameter issues
**Solution**: 
- Verify the parameter file path is correct
- Check YAML syntax with a YAML validator
- Ensure all required parameters are present

### Issue 3: Isaac Sim and Nav2 Time Synchronization
**Problem**: Navigation doesn't work properly due to time issues
**Solution**: 
```bash
# Make sure to set use_sim_time to true in all configurations
# In Isaac Sim, ensure simulation time is being published
```

### Issue 4: Sensor Data Not Available
**Problem**: Nav2 reports missing sensor data
**Solution**: 
- Verify Isaac Sim is publishing required topics
- Check ROS topic list: `ros2 topic list`
- Verify topic names match configuration

## Validation Steps

### Step 1: Verify Installation
```bash
# Check Nav2 packages
ros2 pkg list | grep nav2 | wc -l
# Should return more than 10 packages

# Check our custom package
ros2 pkg list | grep isaac_nav2
```

### Step 2: Check Configuration Files
```bash
# Verify parameter file exists and is valid YAML
ls ~/isaac_nav2_ws/src/isaac_nav2_bringup/config/nav2_isaac_params.yaml

# Check launch file
ls ~/isaac_nav2_ws/src/isaac_nav2_bringup/launch/isaac_navigation_launch.py
```

### Step 3: Test Launch Files (without Isaac Sim running)
```bash
# This will test the syntax of launch files
python3 -m py_compile ~/isaac_nav2_ws/src/isaac_nav2_bringup/launch/isaac_navigation_launch.py
```

## Next Steps

Once the Nav2 setup is complete, you should:

1. **Proceed to Chapter 9.6**: Path Planning Examples with Isaac Tools
2. **Configure your robot model** with appropriate sensors in Isaac Sim
3. **Test navigation** in simple scenarios before moving to complex environments
4. **Tune parameters** based on your specific robot and environment requirements

## Knowledge Check

1. What are the essential Nav2 packages needed for navigation?
2. Why is the `use_sim_time` parameter critical for Isaac integration?
3. What are the required sensor topics for basic navigation?
4. How do you verify that Nav2 packages are properly installed?
5. What is the purpose of the `isaac_nav2_bringup` package?

Answers:
1. Essential packages include: nav2_bringup, nav2_map_server, nav2_planner, nav2_controller, nav2_bt_navigator, nav2_recoveries.
2. The `use_sim_time` parameter ensures synchronization between Isaac Sim's physics simulation and Nav2's timing requirements.
3. Required topics typically include: `/scan` (LiDAR), `/odom` (odometry), `/tf` (transforms), and `/cmd_vel` (velocity commands).
4. Use `ros2 pkg list | grep nav2` to verify installation of Nav2 packages.
5. The `isaac_nav2_bringup` package provides Isaac-specific configurations, launch files, and parameters for Nav2 integration.

## Lab Preparation

Before proceeding to the practical lab exercises in the next section, ensure you have:
- Successfully installed and built the Nav2 packages
- Created and built the `isaac_nav2_bringup` package
- Verified all configuration files are in place
- Understood the integration points between Isaac Sim and Nav2
- Prepared a basic Isaac Sim environment with a robot model