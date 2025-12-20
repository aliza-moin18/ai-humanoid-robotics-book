# VSLAM Setup Procedures in Isaac Sim

## Overview

This document provides detailed procedures for setting up Visual Simultaneous Localization and Mapping (VSLAM) systems using Isaac Sim and Isaac ROS Visual SLAM package. These procedures enable humanoid robots to build maps of their environment while simultaneously determining their position within those maps.

## Prerequisites

### System Requirements
- NVIDIA GPU with RTX capabilities (minimum RTX 2070, recommended RTX 3080 or higher)
- Ubuntu 22.04 LTS
- Isaac Sim 2023.1.1 installed
- Isaac ROS packages (specifically visual_slam) installed and built
- ROS 2 Humble Hawksbill
- CUDA 12.x compatible with your GPU drivers

### Software Dependencies
- Python 3.8-3.10
- Appropriate GPU drivers supporting CUDA
- Isaac ROS Visual SLAM package
- Isaac ROS Image Pipeline package

## VSLAM Setup Procedures

### Procedure 1: Environment Preparation

1. **Verify System Compatibility**
   ```bash
   # Check Isaac Sim installation
   cd /opt/nvidia/isaac_sim
   ./python.sh -c "import omni; print('Isaac Sim accessible')"
   
   # Verify Isaac ROS workspace
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 pkg list | grep visual_slam
   
   # Check GPU access
   python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}'); print(f'GPU count: {torch.cuda.device_count()}')"
   ```

2. **Create VSLAM Workspace Directory**
   ```bash
   mkdir -p ~/isaac_vslam_setup/{config,launch,scripts}
   ```

3. **Set up Environment Variables**
   ```bash
   # Add to ~/.bashrc
   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
   echo 'source ~/isaac_ros_ws/install/setup.bash' >> ~/.bashrc
   echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
   source ~/.bashrc
   ```

### Procedure 2: Isaac Sim Configuration for VSLAM

1. **Select or Create a Robot with Camera Sensors**
   - Open Isaac Sim
   - If using an existing robot, verify it has RGB camera sensors
   - If creating a new robot:
     - Ensure the robot has a calibrated RGB camera
     - Verify the camera is properly positioned (forward-facing for navigation)
     - Check camera parameters (resolution, frame rate, distortion)

2. **Configure Camera Settings in Isaac Sim**
   In Isaac Sim, ensure your camera has appropriate properties:
   - Resolution: Minimum 640x480 (higher for better accuracy)
   - Frame Rate: 15-30 FPS for VSLAM
   - Distortion: Realistic camera distortion coefficients
   - Mounting: Secure, vibration-free mounting

3. **Enable ROS Bridge Extension**
   - In Isaac Sim, go to Window → Extensions
   - Find "ROS Bridge" or "omni.isaac.ros_bridge"
   - Enable the extension
   - Verify the extension shows as active

### Procedure 3: Isaac ROS Visual SLAM Node Configuration

1. **Create VSLAM Parameters File**
   ```bash
   # Create parameters file
   cat > ~/isaac_vslam_setup/config/vslam_params.yaml << EOF
visual_slam_node:
  ros__parameters:
    # Enable occupancy map generation for navigation
    enable_occupancy_map: true
    
    # Resolution of the occupancy map in meters
    occupancy_map_resolution: 0.05
    
    # Size of the map in meters
    occupancy_map_size: 20.0
    
    # Sensor QoS setting
    sensor_qos: 1
    
    # Enable localization mode (for known map scenarios)
    enable_localization: false
    
    # Sensor parameters
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    publish_odom_tf: true
    
    # Max range for valid depth measurements
    depth_image_unit_scaling: 0.001
    
    # Processing parameters
    min_num_features: 100
    max_num_features: 1000
    feature_quality_level: 0.01
    feature_pyramid_level: 4
EOF
   ```

2. **Create VSLAM Launch File**
   ```bash
   # Create basic launch file
   cat > ~/isaac_vslam_setup/launch/vslam.launch.py << EOF
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    # Isaac ROS Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            '/home/\$USER/isaac_vslam_setup/config/vslam_params.yaml'
        ],
        remappings=[
            ('/camera/left/image_rect', '/camera/image_rect_color'),
            ('/camera/left/camera_info', '/camera/camera_info'),
            ('/camera/right/image_rect', '/camera/image_rect_color'),  # For mono/stereo
            ('/camera/right/camera_info', '/camera/camera_info'),
            ('/imu', '/imu/data'),  # If available
        ]
    )
    
    # Image rectification node (if needed)
    rectify_node = Node(
        package='isaac_ros_image_proc',
        executable='isaac_ros_rectify_node',
        name='rectify_node',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
            ('image_rect', '/camera/image_rect_color'),
        ]
    )

    return LaunchDescription([
        rectify_node,
        visual_slam_node
    ])
EOF
   ```

### Procedure 4: Integration with Isaac Sim

1. **Configure Isaac Sim ROS Bridge Settings**
   In Isaac Sim, ensure the ROS bridge is configured to:
   - Publish camera images to `/camera/image_raw`
   - Publish camera info to `/camera/camera_info`
   - Publish appropriate transforms for the robot base and camera
   - Use consistent frame IDs that match your VSLAM configuration

2. **Launch Isaac Sim with Robot**
   ```bash
   cd /opt/nvidia/isaac_sim
   ./isaac-sim.sh --exec "omni.isaac.examples.simple_robots.carter_franka_pick_place"
   ```

3. **Verify Isaac Sim Publishes Required Topics**
   ```bash
   # In a new terminal
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 topic list | grep -E "(camera|camera_info|tf)"
   ```

### Procedure 5: Launch and Test VSLAM System

1. **Launch VSLAM Nodes**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 launch ~/isaac_vslam_setup/launch/vslam.launch.py
   ```

2. **Monitor VSLAM Performance**
   ```bash
   # Check if VSLAM node is publishing pose
   ros2 topic echo /visual_slam/pose --field pose.position.x | head -n 5
   
   # Monitor processing rate
   ros2 topic hz /camera/image_rect_color
   
   # Check GPU utilization
   watch -n 1 nvidia-smi
   ```

3. **Test VSLAM in Isaac Sim**
   - In Isaac Sim, move the robot around the environment
   - Monitor the VSLAM pose output in the terminal
   - Check that the robot's estimated position follows its actual movement in simulation
   - Verify the occupancy map is being generated if enabled

### Procedure 6: Advanced Configuration

1. **Optimize for Performance**
   - Adjust the number of features to track based on your computational resources:
     ```yaml
     visual_slam_node:
       ros__parameters:
         min_num_features: 50      # Lower for performance
         max_num_features: 500     # Lower for performance
     ```
   - Reduce map resolution if detailed maps aren't needed:
     ```yaml
     occupancy_map_resolution: 0.1  # Higher value for less detail but better performance
     ```

2. **Enable Loop Closure (Optional)**
   Add these parameters to your config file for improved accuracy:
   ```yaml
   visual_slam_node:
     ros__parameters:
       # Loop closure parameters
       enable_local_map_for_tracking: true
       enable_local_map_for_localization: true
       local_map_size: 10.0
   ```

3. **Configure for Different Camera Types**
   For stereo cameras, modify your launch file:
   ```python
   # In your launch file, replace camera remappings:
   remappings=[
       ('/camera/left/image_rect', '/camera/left/image_rect'),
       ('/camera/left/camera_info', '/camera/left/camera_info'),
       ('/camera/right/image_rect', '/camera/right/image_rect'),
       ('/camera/right/camera_info', '/camera/right/camera_info'),
   ]
   ```

## Troubleshooting VSLAM Setup Issues

### Common Issues and Solutions

**Issue 1: VSLAM node doesn't receive camera images**
- **Check**: Verify Isaac Sim camera is publishing properly
- **Solution**: 
  ```bash
  ros2 topic echo /camera/image_raw | head -n 5
  ```
  If no data, verify Isaac Sim camera configuration and ROS bridge setup.

**Issue 2: High processing latency**
- **Check**: Monitor GPU utilization
- **Solution**: Reduce feature count or image resolution in parameters

**Issue 3: Drifting pose estimates**
- **Check**: Verify camera calibration and robot movement speed
- **Solution**: Slow robot movement, verify camera calibration parameters

**Issue 4: VSLAM doesn't initialize**
- **Check**: Verify all required topics are available
- **Solution**: Ensure camera_info and image topics are publishing

### Debugging Steps

1. **Check All Required Topics**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 topic list | grep -E "(camera|info|tf|visual_slam)"
   ```

2. **Monitor VSLAM Node Status**
   ```bash
   ros2 component list  # If using composable nodes
   ros2 lifecycle list visual_slam_node  # If using lifecycle nodes
   ```

3. **View Transform Tree**
   ```bash
   ros2 run tf2_tools view_frames
   # This creates frames.pdf showing the transform tree
   ```

## Validation Procedures

### Basic Validation
1. **Check Node Status**:
   ```bash
   ros2 node list | grep visual_slam
   ```

2. **Verify Topic Publications**:
   ```bash
   ros2 topic list | grep visual_slam
   ros2 topic info /visual_slam/pose
   ```

3. **Test Pose Output**:
   ```bash
   ros2 topic echo /visual_slam/pose | head -n 10
   ```

### Performance Validation
1. **Monitor Processing Rate**:
   ```bash
   ros2 topic hz /visual_slam/pose
   ```

2. **Check Resource Usage**:
   ```bash
   # Monitor GPU usage
   nvidia-smi -l 1
   ```

3. **Validate Map Generation** (if enabled):
   ```bash
   ros2 topic echo /visual_slam/occupancy_grid --field info.width
   ```

## Optimization Tips

### Performance Optimization
1. **Feature Count**: Adjust min/max features based on scene complexity
2. **Image Resolution**: Lower resolution for better performance
3. **Processing Frequency**: Match to robot mobility (if moving slowly, process less frequently)

### Accuracy Optimization
1. **Camera Quality**: Higher resolution and better calibration
2. **Scene Texture**: Ensure environment has sufficient visual features
3. **Lighting Consistency**: Avoid scenes with changing lighting conditions

## Integration with AI-Robot Brain System

The VSLAM system integrates into the AI-robot brain as follows:
1. **Perception Layer**: Provides environmental understanding
2. **Navigation System**: Supplies position and map data
3. **Control System**: Provides awareness for safe movement
4. **Learning System**: Generates training data for sim-to-real transfer

## Best Practices

1. **Consistent Frame IDs**: Use standard ROS frame conventions
2. **Parameter Validation**: Test with various parameter configurations
3. **Error Handling**: Implement fallback behaviors when VSLAM fails
4. **Resource Monitoring**: Continuously monitor GPU and CPU usage
5. **Calibration**: Regularly verify camera calibration parameters

## Troubleshooting Commands Summary

```bash
# Essential debugging commands
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# Check node status
ros2 node list
ros2 lifecycle list visual_slam_node

# Check topics
ros2 topic list
ros2 topic info /camera/image_raw
ros2 topic info /visual_slam/pose

# Monitor data flow
ros2 topic echo /visual_slam/pose --field pose.position
ros2 topic hz /camera/image_rect_color

# Check transforms
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link

# View computational resources
nvidia-smi
```

This setup procedure provides a complete workflow for implementing VSLAM in Isaac Sim, from basic setup through advanced optimization and validation.