# Troubleshooting Guide for Common Isaac Setup Issues

## Overview

This guide provides solutions for common issues encountered during the setup and configuration of NVIDIA Isaac Sim and Isaac ROS. Following these troubleshooting steps will help resolve most common problems that occur during the initial setup process.

## System Prerequisites Verification

### Check GPU Compatibility
**Issue**: Isaac Sim or Isaac ROS fails to start or uses CPU only
**Solution**:
1. Verify your GPU is NVIDIA and supports CUDA:
   ```bash
   nvidia-smi
   ```
2. Check CUDA compute capability (minimum 6.0):
   ```bash
   nvidia-ml-py3 -c "import pynvml; pynvml.nvmlInit(); handle = pynvml.nvmlDeviceGetHandleByIndex(0); print(pynvml.nvmlDeviceGetName(handle).decode('utf-8'))"
   ```
3. Ensure GPU drivers are up to date and CUDA version is 11.8 or 12.x

### Verify Ubuntu Version
**Issue**: Installation fails or components don't work as expected
**Solution**:
```bash
lsb_release -a
```
Ensure you are running Ubuntu 22.04 LTS. If not, consider upgrading or using the supported Linux distribution.

### Check Available Disk Space
**Issue**: Installation fails due to insufficient space
**Solution**:
```bash
df -h
```
Ensure at least 100GB free space is available for Isaac Sim installation, plus additional space for simulation assets.

## Isaac Sim Installation Issues

### Isaac Sim Fails to Launch
**Issue**: Isaac Sim crashes immediately on startup
**Solutions**:

1. **Graphics Driver Issues**:
   - Update NVIDIA drivers to the latest version from NVIDIA's website
   - Check compatibility with your GPU model
   - Ensure OpenGL 4.5 or higher is supported

2. **Display Configuration**:
   - If no monitor is connected, try launching with:
   ```bash
   cd /opt/nvidia/isaac_sim
   ./isaac-sim.sh --no-window
   ```

3. **Permissions Issues**:
   - Check Isaac Sim directory permissions:
   ```bash
   ls -la /opt/nvidia/isaac_sim
   sudo chown -R $USER:$USER /opt/nvidia/isaac_sim
   ```

4. **Library Dependencies**:
   - Install missing dependencies:
   ```bash
   sudo apt update
   sudo apt install libglib2.0-0 libsm6 libxext6 libxrender-dev libgomp1
   ```

### Python API Issues
**Issue**: Isaac Sim Python API is not accessible
**Solutions**:

1. **Environment Variables**:
   - Ensure ISAACSIM environment variables are set:
   ```bash
   echo $ISAACSIM_PATH
   echo $ISAACSIM_PYTHON_EXE
   ```
   
2. **Use Isaac Sim's Python**:
   - Always use Isaac Sim's Python executable for API access:
   ```bash
   /opt/nvidia/isaac_sim/python.sh -c "import omni; print('Success')"
   ```

3. **Virtual Environment Conflicts**:
   - Deactivate any conda/virtual environments before using Isaac Sim Python
   - Isaac Sim comes with its own Python environment

### Isaac Sim Performance Problems
**Issue**: Low frame rates or lag in simulation
**Solutions**:

1. **Check GPU Utilization**:
   ```bash
   nvidia-smi
   ```
   If GPU utilization is low, the bottleneck might be CPU or memory-related

2. **Reduce Simulation Quality**:
   - In Isaac Sim settings, reduce rendering quality
   - Lower physics update rate in simulation settings
   - Simplify scene complexity

3. **Memory Issues**:
   - Monitor system memory usage: `htop`
   - Close other GPU-intensive applications
   - Check for sufficient VRAM in `nvidia-smi`

## Isaac ROS Installation Issues

### Isaac ROS Packages Fail to Build
**Issue**: `colcon build` fails with Isaac ROS packages
**Solutions**:

1. **Missing Dependencies**:
   ```bash
   cd ~/isaac_ros_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **ROS 2 Environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ```

3. **CUDA Compatibility**:
   - Verify CUDA version compatibility with Isaac ROS
   - Check that CUDA libraries are accessible:
   ```bash
   nvcc --version
   echo $CUDA_HOME
   ```

4. **Build Parallelism**:
   - If running out of memory during build:
   ```bash
   colcon build --symlink-install --parallel-workers 2
   ```

### Isaac ROS Nodes Fail to Initialize
**Issue**: Isaac ROS nodes crash or fail to start
**Solutions**:

1. **GPU Access**:
   - Verify GPU is accessible to Isaac ROS:
   ```bash
   python3 -c "import torch; print(torch.cuda.is_available())"
   ```

2. **Permissions**:
   - Add user to video and render groups:
   ```bash
   sudo usermod -a -G video $USER
   sudo usermod -a -G render $USER
   # Log out and log back in for changes to take effect
   ```

3. **Memory Issues**:
   - Check GPU memory usage: `nvidia-smi`
   - Isaac ROS packages require significant GPU memory for operation

## ROS 2 Integration Issues

### Isaac Sim and ROS 2 Don't Communicate
**Issue**: Isaac Sim doesn't publish to ROS 2 topics or vice versa
**Solutions**:

1. **ROS Bridge Extension**:
   - In Isaac Sim, go to Window → Extensions
   - Search for "ROS Bridge" or "omni.isaac.ros_bridge"
   - Enable the extension if not already enabled

2. **Environment Variables**:
   ```bash
   echo $RMW_IMPLEMENTATION
   echo $ROS_DOMAIN_ID
   ```
   Ensure consistency between Isaac Sim and ROS 2 environments

3. **Network Configuration**:
   - Check that DDS is working properly
   - Ensure no firewall is blocking ROS 2 communication
   - Verify both Isaac Sim and ROS 2 nodes are on the same ROS_DOMAIN_ID

### ROS 2 Topics Not Appearing
**Issue**: Can't see Isaac Sim topics with `ros2 topic list`
**Solutions**:

1. **Check Isaac Sim Simulation**:
   - Ensure simulation is running in Isaac Sim
   - Verify sensors are properly configured and publishing

2. **ROS 2 Environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ```

3. **DDS Communication**:
   - The default RMW implementation may cause issues
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedxs_cpp
   ```

## Common Runtime Issues

### Isaac ROS Nodes Have High Latency
**Issue**: Isaac ROS perception nodes have high processing latency
**Solutions**:

1. **NITROS Configuration**:
   - Verify NITROS is properly configured for optimized data transfer
   - Check Quality of Service (QoS) settings between nodes

2. **GPU Utilization**:
   - Monitor GPU usage during processing
   - Ensure GPU isn't being used by other processes

3. **Sensor Data Rate**:
   - Reduce sensor publishing rate in Isaac Sim if too high
   - Match Isaac ROS processing capabilities with sensor rate

### Transform (tf2) Issues
**Issue**: Transform lookup fails between robot frames
**Solutions**:

1. **Check Transform Publishing**:
   ```bash
   ros2 run tf2_tools view_frames
   ```

2. **Verify Robot URDF**:
   - Ensure URDF is properly loaded
   - Check that joint states are being published

3. **Timing Issues**:
   - Verify Isaac Sim and ROS 2 clocks are synchronized
   - Check that Isaac Sim is actively simulating

## Hardware-Specific Troubleshooting

### Issues with Jetson Platforms
For Jetson edge devices:

1. **Thermal Throttling**:
   - Monitor temperature: `sudo tegrastats`
   - Ensure adequate cooling is provided

2. **Power Mode**:
   - Set appropriate power mode for your application
   ```bash
   sudo nvpmodel -m 0  # Maximum performance mode
   ```

3. **Memory Constraints**:
   - Monitor both CPU and GPU memory usage
   - Adjust Isaac ROS parameters for lower resource usage

### Issues with RTX GPUs
For RTX GPU-specific issues:

1. **RT Cores and Tensor Cores**:
   - Verify Isaac ROS is utilizing RT and Tensor Cores
   - Check Isaac ROS documentation for RTX-specific optimizations

2. **VRAM Exhaustion**:
   - Monitor VRAM usage: `nvidia-smi`
   - Reduce simulation complexity or sensor resolution

## Verification and Testing

### Quick Verification Commands
Run these commands to verify basic functionality:

1. **Isaac Sim Access**:
   ```bash
   cd /opt/nvidia/isaac_sim
   ./python.sh -c "import omni; print('Isaac Sim API accessible')"
   ```

2. **ROS 2 with Isaac Packages**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 pkg list | grep isaac
   ```

3. **GPU Access from Isaac ROS**:
   ```bash
   source ~/isaac_ros_ws/install/setup.bash
   python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
   ```

### System Monitoring
Use these tools to monitor system during operation:

1. **GPU and System Resources**:
   ```bash
   watch -n 1 nvidia-smi
   ```

2. **ROS 2 Communication**:
   ```bash
   rqt_graph
   ```

3. **ROS 2 Topic Rates**:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list | xargs -I {} ros2 topic hz {}
   ```

## Advanced Troubleshooting

### Log Analysis
Isaac Sim logs are typically located at:
- `~/.nvidia-omniverse/logs/isaac-sim/`
- `/opt/nvidia/isaac_sim/logs/`

ROS 2 nodes log to the terminal where they are launched.

### Common Error Patterns
- **CUDA Initialization Errors**: Usually GPU access or driver issues
- **DDS Communication Errors**: Network or RMW configuration issues
- **Memory Allocation Errors**: Insufficient system or GPU memory
- **Extension Loading Errors**: Isaac Sim configuration issues

## Support Resources

If the above troubleshooting steps do not resolve your issue:

1. Check the NVIDIA Isaac documentation at: https://nvidia-isaac-ros.github.io/
2. Review the Isaac Sim documentation: https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html
3. For ROS 2 issues: https://docs.ros.org/
4. Check the Isaac ROS GitHub repositories for known issues
5. Search the NVIDIA Developer Forums for similar issues

## Preventive Measures

To minimize setup issues:

1. Perform a clean installation on a fresh Ubuntu 22.04 system if possible
2. Ensure all prerequisites are met before beginning installation
3. Follow the installation steps exactly as outlined in the documentation
4. Verify each step before proceeding to the next
5. Keep system updated with the latest NVIDIA drivers and system patches