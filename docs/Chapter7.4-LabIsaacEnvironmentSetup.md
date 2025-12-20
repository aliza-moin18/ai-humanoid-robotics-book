# Chapter 7.4: Lab: Isaac Environment Setup

## Objective

Successfully set up the complete NVIDIA Isaac Sim and Isaac ROS environment on a development workstation and verify proper integration with ROS 2. Students will complete the setup process and validate that both Isaac Sim and Isaac ROS environments are properly configured and functional.

## Materials Required

### Hardware
- NVIDIA RTX 3080 or higher (RTX 4090 recommended)
- Ubuntu 22.04 LTS
- 32GB RAM minimum (64GB recommended)
- 1TB SSD storage for simulation assets

### Software
- NVIDIA GPU drivers (latest version supporting CUDA 12.x)
- Isaac Sim 2023.1.1
- ROS 2 Humble Hawksbill
- Isaac ROS 3.1
- CUDA 12.x
- Omniverse Kit

## Setup Instructions

### Step 1: Verify System Prerequisites
1. Check Ubuntu version:
   ```bash
   lsb_release -a
   ```
   Ensure you have Ubuntu 22.04 LTS.

2. Check NVIDIA GPU and driver:
   ```bash
   nvidia-smi
   ```
   Verify your GPU is detected and the driver version supports CUDA 12.x.

3. Check available disk space:
   ```bash
   df -h
   ```
   Ensure at least 100GB free space for Isaac Sim installation.

### Step 2: Install ROS 2 Humble Hawksbill
1. Update system packages:
   ```bash
   sudo apt update && sudo apt upgrade
   ```

2. Set locale:
   ```bash
   locale  # Check if LANG=en_US.UTF-8 is set
   sudo locale-gen en_US.UTF-8
   ```

3. Add ROS 2 GPG key and repository:
   ```bash
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

4. Install ROS 2:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-colcon-common-extensions
   ```

5. Source ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

### Step 3: Install Isaac Sim
1. Download Isaac Sim 2023.1.1 from NVIDIA Developer website
2. Extract the downloaded package to the default location: `/opt/nvidia/isaac_sim`
   ```bash
   # Assuming the downloaded file is isaac-sim-2023.1.1.tar.gz
   cd /opt/nvidia
   sudo tar -xzf /path/to/isaac-sim-2023.1.1.tar.gz
   # Or follow the installer instructions from NVIDIA
   ```

2. Check the installation folder structure:
   ```bash
   ls -la /opt/nvidia/isaac_sim
   ```
   You should see folders like `apps`, `exts`, `isaacsim`, etc.

### Step 4: Install Isaac ROS Packages
1. Create a workspace for Isaac ROS:
   ```bash
   mkdir -p ~/isaac_ros_ws/src
   cd ~/isaac_ros_ws
   ```

2. Clone the Isaac ROS common repository:
   ```bash
   cd ~/isaac_ros_ws/src
   git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
   git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
   git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
   git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_stereo_image_proc.git
   ```

3. Install dependencies:
   ```bash
   cd ~/isaac_ros_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the workspace:
   ```bash
   cd ~/isaac_ros_ws
   colcon build --symlink-install --packages-select isaac_ros_common
   source install/setup.bash
   ```

### Step 5: Configure ROS 2 and Isaac Integration
1. Source both ROS 2 and Isaac ROS environments:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ```

2. Verify Isaac Sim can be launched:
   ```bash
   cd /opt/nvidia/isaac_sim
   ./isaac-sim.sh
   # Or if using the launcher:
   # python3 -m omni.isaac.sim.python._cli.isaacsim --enable_ros2
   ```

### Step 6: Verify Isaac ROS Package Installation
1. Check available Isaac ROS packages:
   ```bash
   ros2 pkg list | grep isaac
   ```

2. Verify Isaac ROS bridge functionality:
   ```bash
   ros2 run isaac_ros_apriltag isaac_ros_apriltag_node
   ```

### Step 7: Test Basic Simulation
1. Launch Isaac Sim with a simple example:
   ```bash
   cd /opt/nvidia/isaac_sim
   ./isaac-sim.sh --exec "omni.isaac.examples.simple_robots.carter_franka_pick_place"
   ```

2. Verify that the simulation environment loads correctly with the Carter robot and Franka manipulator.

### Step 8: Test ROS 2 Integration
1. In Isaac Sim, enable the ROS Bridge extension:
   - Go to Window → Extensions
   - Search for "ROS Bridge" or "omni.isaac.ros_bridge"
   - Enable the extension

2. Run a simple simulation and verify ROS 2 topics are being published:
   ```bash
   # In a new terminal
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 topic list
   ```

3. Verify specific topics like `/camera/image_raw` or `/scan` if using appropriate sensors in the simulation.

## Procedures

### Procedure 1: Verify Isaac Sim Installation
1. Launch Isaac Sim from the terminal:
   ```bash
   cd /opt/nvidia/isaac_sim
   ./isaac-sim.sh
   ```

2. Expected outcome: Isaac Sim application should start without errors, showing the main menu or a default scene.

3. Troubleshooting:
   - If Isaac Sim fails to start, check GPU driver compatibility
   - Look for error messages in the terminal output
   - Verify sufficient disk space and permissions

### Procedure 2: Verify Isaac ROS Package Functionality
1. Launch an Isaac ROS package in a terminal:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 run isaac_ros_visual_slam isaac_ros_visual_slam_node
   ```

2. Expected outcome: The node should start without errors and show initialization messages.

3. Troubleshooting:
   - Check CUDA version compatibility with Isaac ROS
   - Verify GPU is accessible to the Isaac ROS node
   - Ensure Isaac ROS packages were built correctly

### Procedure 3: Test Isaac Sim and ROS 2 Integration
1. Set up environment variables for ROS 2 integration in Isaac Sim:
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export ROS_DOMAIN_ID=0
   ```

2. Launch Isaac Sim with ROS 2 bridge enabled:
   ```bash
   cd /opt/nvidia/isaac_sim
   ./isaac-sim.sh --exec "omni.isaac.ros_bridge.scripts.isaac_sim_bridge"
   ```

3. In a separate terminal, check for published topics:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ```

4. Expected outcome: Should see topics published from Isaac Sim (e.g., `/robot_description`, `/joint_states`, sensor topics).

### Procedure 4: Run Complete Integration Test
1. Create a simple launch file to test integration:
   ```bash
   # Create a new directory for our test
   mkdir -p ~/isaac_robotics_test/launch
   ```

2. Create a launch file (~/isaac_robotics_test/launch/test_integration.launch.py):
   ```python
   from launch import LaunchDescription
   from launch_ros.actions import Node
   from launch.actions import ExecuteProcess
   import os

   def generate_launch_description():
       # Launch Isaac Sim
       isaac_sim_cmd = ExecuteProcess(
           cmd=['/opt/nvidia/isaac_sim/isaac-sim.sh', '--no-window', '--exec', 'omni.isaac.examples.simple_robots.carter_franka_pick_place'],
           output='screen'
       )
       
       # Launch Isaac ROS visual SLAM node
       visual_slam_node = Node(
           package='isaac_ros_visual_slam',
           executable='isaac_ros_visual_slam_node',
           name='visual_slam_node',
           parameters=[{
               'enable_occupancy_map': True,
               'occupancy_map_resolution': 0.05,
           }]
       )
       
       return LaunchDescription([
           isaac_sim_cmd,
           visual_slam_node
       ])
   ```

3. Launch the integration test:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 launch ~/isaac_robotics_test/launch/test_integration.launch.py
   ```

4. Expected outcome: Both Isaac Sim and Isaac ROS nodes should run without errors, with data being processed between them.

## Assessment Criteria

### Completion Requirements
- [ ] Successfully install Isaac Sim 2023.1.1
- [ ] Successfully install ROS 2 Humble Hawksbill
- [ ] Successfully install and build Isaac ROS packages
- [ ] Verify Isaac Sim launches without errors
- [ ] Confirm Isaac ROS packages run without errors
- [ ] Demonstrate successful integration between Isaac Sim and ROS 2
- [ ] Show published topics from Isaac Sim in ROS 2
- [ ] Complete the integration test launch file

### Performance Metrics
- [ ] Setup completion within 2 hours as specified in success criteria
- [ ] No critical errors during installation process
- [ ] All required components properly configured and functional
- [ ] Successful verification of Isaac Sim and Isaac ROS integration

### Verification Steps
1. Run the Isaac Sim verification script:
   ```bash
   cd /opt/nvidia/isaac_sim
   python3 -c "import omni; print('Isaac Sim Python API accessible')"
   ```

2. Check Isaac ROS functionality:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 run isaac_ros_apriltag isaac_ros_apriltag_node --ros-args --log-level info
   ```

3. Verify integration:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list | grep -E "(camera|scan|imu)"
   ```

## Lab Report Requirements

Students must submit a lab report documenting:

1. **Installation Process**: Step-by-step account of the installation process, including any challenges encountered and solutions applied.
2. **Verification Results**: Output from verification commands and screenshots of successful installations.
3. **Integration Test Results**: Details of the integration test, including which components were tested and the results.
4. **Troubleshooting**: Any issues encountered during setup and how they were resolved.
5. **System Specifications**: Hardware and software specifications of the system used for installation.

## Troubleshooting Guide

### Common Issues and Solutions

1. **Isaac Sim fails to start with graphics errors**
   - Ensure NVIDIA GPU drivers are properly installed
   - Verify OpenGL support in your system
   - Check for sufficient VRAM availability

2. **Isaac ROS packages fail to build**
   - Check CUDA version compatibility
   - Ensure all dependencies are installed
   - Verify system has sufficient storage space

3. **ROS 2 nodes cannot communicate with Isaac Sim**
   - Verify ROS_DOMAIN_ID is consistent between Isaac Sim and ROS 2
   - Check firewall settings if running across different machines
   - Ensure RMW_IMPLEMENTATION is set correctly

4. **GPU not accessible to Isaac ROS packages**
   - Verify CUDA installation and GPU driver
   - Check user permissions for GPU access
   - Ensure Isaac ROS packages are compatible with installed CUDA version

## Extensions

### Advanced Setup
For advanced students, try:
1. Setting up Isaac Sim with Docker for containerized environments
2. Configuring Isaac Sim to work with different ROS 2 distributions
3. Installing additional Isaac ROS packages beyond the basic set

### Performance Optimization
1. Configure Isaac Sim for optimal performance on your hardware
2. Set up Isaac ROS for specific use cases (navigation, manipulation, etc.)
3. Optimize quality of service settings for your network configuration