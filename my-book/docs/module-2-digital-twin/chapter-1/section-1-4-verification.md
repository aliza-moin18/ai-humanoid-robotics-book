---
sidebar_position: 4
---

# Section 1.4: Verification of Installation

## Overview

This section provides comprehensive verification procedures to ensure that Gazebo and Unity environments are properly configured and integrated. Students will perform tests to validate their setup before proceeding to physics simulation concepts.

## Verification Checklist

Complete the following tests to verify your installation:

### Test 1: Gazebo Functionality
- [ ] Gazebo launches without errors
- [ ] Basic simulation runs (shapes.sdf example)
- [ ] Physics simulation behaves correctly
- [ ] Camera controls work properly

### Test 2: Unity Functionality
- [ ] Unity Editor launches without errors
- [ ] New project can be created
- [ ] Basic 3D scene renders correctly
- [ ] Physics simulation works in Unity

### Test 3: Integration Verification
- [ ] ROS 2 communication is established
- [ ] Bridge node connects successfully
- [ ] Data flows between Gazebo and Unity
- [ ] Robot model synchronizes between platforms

## Detailed Verification Steps

### Step 1: Gazebo Verification

1. Launch Gazebo:
   ```bash
   gz sim
   ```

2. Load a test world:
   - File > Open > shapes.sdf (or similar test world)
   - Verify that objects fall under gravity
   - Test camera controls (orbit, pan, zoom)

3. Check physics engine:
   ```bash
   gz topic -i -t /world/default/stats
   ```

4. Verify plugins load correctly:
   - Check terminal for error messages
   - Confirm physics engine is DART (for humanoid robots)

### Step 2: Unity Verification

1. Launch Unity Hub and open a new 3D project

2. Create a simple test scene:
   - Add a plane (ground)
   - Add a cube above the plane
   - Add physics components to the cube
   - Play the scene and verify physics simulation

3. Test rendering settings:
   - Verify lighting works correctly
   - Check that materials render properly
   - Test camera movement

### Step 3: Integration Verification

1. Launch ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Start Gazebo with a simple robot model:
   ```bash
   gz sim -v 4 simple_robot.sdf
   ```

3. Launch the bridge node:
   ```bash
   ros2 run gazebo_ros spawn_entity.py -file robot.urdf -entity robot
   ```

4. In Unity, connect to the ROS network and verify:
   - Robot position updates from Gazebo appear in Unity
   - Sensor data (if available) is received
   - Control commands from Unity affect Gazebo simulation

### Step 4: Performance Verification

1. Monitor system resources during simulation
2. Verify that both Gazebo and Unity run at acceptable frame rates
3. Test with a more complex scene to ensure stability

## Common Issues and Solutions

### Issue 1: Gazebo fails to launch
- **Symptom**: Gazebo crashes on startup
- **Solution**: Check graphics drivers and install required libraries:
  ```bash
  sudo apt install mesa-utils
  glxinfo | grep "OpenGL version"
  ```

### Issue 2: Unity fails to render
- **Symptom**: Unity shows black screen or rendering errors
- **Solution**: Update graphics drivers and check Unity system requirements

### Issue 3: ROS 2 communication fails
- **Symptom**: No messages passing between nodes
- **Solution**: Verify ROS_DOMAIN_ID and network configuration:
  ```bash
  echo $ROS_DOMAIN_ID
  ros2 topic list
  ```

### Issue 4: Integration latency
- **Symptom**: Delay between Gazebo and Unity updates
- **Solution**: Optimize update rates and check network performance

## Success Criteria

Your installation is successfully verified when:
- All checklist items are marked as complete
- All verification steps pass without errors
- Performance meets minimum requirements (30+ FPS in both environments)
- Integration demonstrates real-time synchronization

## Troubleshooting Resources

- Check the [Gazebo Troubleshooting Guide](https://gazebosim.org/docs/harmonic/troubleshooting)
- Review the [Unity Troubleshooting Guide](https://docs.unity3d.com/Manual/Troubleshooting.html)
- Consult the [ROS 2 Troubleshooting Guide](https://docs.ros.org/en/humble/Troubleshooting.html)

## Next Steps

After successfully verifying your installation, you're ready to proceed to [Chapter 2: Physics Simulation Fundamentals](../chapter-2/index) where you'll learn about physics simulation concepts for digital twin applications.