# Assessment Questions for Isaac Setup Validation

## Overview

This assessment evaluates students' ability to successfully set up and verify the NVIDIA Isaac Sim and Isaac ROS environments. The questions test understanding of the installation process, configuration, and verification procedures covered in the Isaac Environment Setup lab.

## Pre-Installation Assessment

### Question 1: Hardware Requirements
**Multiple Choice**
Which of the following GPU configurations meets the minimum requirements for Isaac Sim?
A) NVIDIA GTX 1060 with 6GB VRAM
B) NVIDIA RTX 2070 with 8GB VRAM
C) AMD Radeon RX 6700 XT with 12GB VRAM
D) Intel UHD Graphics 630

**Answer: B** - RTX 2070 with 8GB VRAM is the minimum recommended GPU for Isaac Sim

### Question 2: System Prerequisites
**True/False**
Isaac Sim 2023.1.1 can be installed on Ubuntu 20.04 LTS without compatibility issues.

**Answer: False** - Isaac Sim 2023.1.1 requires Ubuntu 22.04 LTS for optimal compatibility

### Question 3: Software Dependencies
**Short Answer**
List the three critical software components that must be installed before Isaac Sim.

**Answer:**
1. NVIDIA GPU drivers (version 535.0 or later)
2. CUDA Toolkit (version 12.x)
3. Ubuntu 22.04 LTS

## Installation Process Assessment

### Question 4: Installation Steps
**Multiple Choice**
What is the default installation location for Isaac Sim when following standard installation procedures?
A) ~/isaac_sim
B) /usr/local/isaac_sim
C) /opt/nvidia/isaac_sim
D) /home/user/isaac_sim

**Answer: C** - The default location is /opt/nvidia/isaac_sim

### Question 5: Isaac ROS Workspace
**Short Answer**
Explain the purpose of creating a separate ROS workspace for Isaac ROS packages and describe the standard directory structure.

**Answer:**
The Isaac ROS workspace provides isolation from other ROS projects and contains all Isaac ROS packages in a single build environment. The standard structure is: `~/isaac_ros_ws/src/` for source code and `~/isaac_ros_ws/install/` for built packages.

### Question 6: Isaac ROS Package Installation
**Multiple Choice**
Which command correctly clones the Isaac ROS common package?
A) git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
B) git clone -b ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
C) git clone --ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
D) git clone ros2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

**Answer: B** - Using the `-b ros2` flag ensures you get the ROS 2 compatible branch

### Question 7: Build Process
**Multiple Choice**
Which command properly builds only the Isaac ROS common package?
A) colcon build
B) colcon build --packages-select isaac_ros_common
C) colcon build --only-isaac_ros_common
D) build isaac_ros_common

**Answer: B** - The `--packages-select` flag builds only the specified package

## Configuration Assessment

### Question 8: Environment Variables
**Fill in the Blank**
Complete the bashrc entry to source the Isaac ROS workspace:
```bash
source ________/install/setup.bash
```

**Answer:** `~/isaac_ros_ws`

### Question 9: ROS 2 Configuration
**Short Answer**
Why is it recommended to set the RMW_IMPLEMENTATION to rmw_cyclonedds_cpp for Isaac ROS?

**Answer:**
CycloneDDS provides better real-time and embedded system support, which is important for Isaac ROS's low-latency requirements and deterministic behavior needed in robotics applications.

### Question 10: QoS Settings
**True/False**
Quality of Service (QoS) settings are not important when connecting Isaac Sim sensors to Isaac ROS processing nodes.

**Answer: False** - QoS settings are critical for ensuring proper data flow between Isaac Sim and Isaac ROS nodes, especially for real-time processing.

## Verification Assessment

### Question 11: Isaac Sim Verification
**Multiple Choice**
Which command correctly tests the Isaac Sim Python API accessibility?
A) python3 -c "import isaac"
B) /opt/nvidia/isaac_sim/python.sh -c "import omni"
C) python3 -c "import omni"
D) isaac_sim_test.py

**Answer: B** - Using Isaac Sim's own Python executable is required for API access

### Question 12: Isaac ROS Verification
**Multiple Choice**
Which command would best verify that Isaac ROS packages are recognized by ROS 2?
A) ls ~/isaac_ros_ws/src
B) ros2 pkg list | grep isaac
C) ros2 topic list
D) ~/isaac_ros_ws/install/setup.bash

**Answer: B** - This command lists all available packages and filters for Isaac packages

### Question 13: GPU Access Verification
**Short Answer**
Describe the command to verify that CUDA is accessible from a Python environment after Isaac ROS installation.

**Answer:**
```bash
python3 -c "import torch; print(torch.cuda.is_available())"
```
This should return `True` if CUDA is properly accessible.

## Troubleshooting Assessment

### Question 14: Common Issues
**Multiple Choice**
If Isaac Sim fails to launch with graphics errors, which of the following should be checked first?
A) The version of Ubuntu
B) The installation of CUDA
C) The NVIDIA GPU drivers
D) The amount of system RAM

**Answer: C** - Graphics errors typically indicate issues with GPU drivers or compatibility

### Question 15: Isaac ROS Issues
**Multiple Choice**
What is the most likely cause if Isaac ROS packages build successfully but nodes fail to launch?
A) Incorrect Isaac Sim version
B) GPU not accessible to the process
C) Wrong Ubuntu version
D) Missing Python 2.7

**Answer: B** - Isaac ROS packages require GPU access for their accelerated computations

## Practical Assessment

### Question 16: Setup Validation
**Practical Task**
Given a system with Isaac Sim and Isaac ROS installed, write the sequence of commands to:
1. Source all necessary environments
2. Verify Isaac Sim installation
3. Verify Isaac ROS packages are recognized
4. Check if GPU is accessible

**Answer:**
```bash
# 1. Source environments
source /opt/ros/humble/setup.bash
source ~/isaac_ros_ws/install/setup.bash

# 2. Verify Isaac Sim
cd /opt/nvidia/isaac_sim
./python.sh -c "import omni; print('Isaac Sim API accessible')"

# 3. Verify Isaac ROS packages
ros2 pkg list | grep isaac

# 4. Check GPU access
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

### Question 17: Integration Verification
**Practical Task**
Explain how to verify that Isaac Sim and ROS 2 are properly integrated by checking for published topics.

**Answer:**
1. Launch Isaac Sim with ROS bridge enabled
2. In a separate terminal, source ROS 2 and Isaac ROS environments:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ```
3. Run `ros2 topic list` and verify Isaac Sim topics are present (e.g., `/camera/image_raw`, `/joint_states`, `/tf`)
4. Optionally, echo a topic to verify data is flowing: `ros2 topic echo /clock --field clock.nanosec`

### Question 18: Performance Validation
**Short Answer**
How would you verify that Isaac ROS packages are utilizing GPU acceleration rather than CPU processing?

**Answer:**
Monitor GPU utilization with `nvidia-smi` while running Isaac ROS nodes. If properly configured, you should see increased GPU usage and memory allocation. Additionally, Isaac ROS nodes will report initialization of GPU-accelerated algorithms in their logs.

## Assessment Rubric

### Scoring Guide
- **Multiple Choice Questions (1, 4, 6, 7, 11, 12, 14, 15, 16)**: 1 point each
- **True/False Questions (2, 10)**: 1 point each
- **Fill in the Blank Question (8)**: 1 point
- **Short Answer Questions (3, 9, 13, 17, 18)**: 2 points each
- **Practical Tasks (16, 17)**: 3 points each

### Total Points: 26 points

### Grading Scale
- **Advanced (A)**: 23-26 points (88-100%)
- **Proficient (B)**: 20-22 points (77-87%)
- **Developing (C)**: 16-19 points (62-76%)
- **Beginning (D)**: 13-15 points (50-61%)
- **Incomplete (F)**: Below 13 points (<50%)

## Success Criteria

Students must achieve at least a "Proficient" (B) level (20/26 points) to demonstrate adequate understanding and capability to proceed with Isaac-based development. This threshold corresponds to the 85% accuracy requirement specified in the success criteria for the AI-Robot Brain module.