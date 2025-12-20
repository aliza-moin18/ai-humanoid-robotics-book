# API and Interface Contracts: AI-Robot Brain (NVIDIA Isaac)

## Overview
This document describes the key interfaces, protocols, and contracts relevant to Module 3: The AI-Robot Brain. These contracts define the expected interactions between different components of the NVIDIA Isaac ecosystem, including Isaac Sim, Isaac ROS, and Nav2.

## ROS 2 Message Interfaces

### Common Message Types
- **sensor_msgs/Image**: Camera image data from simulated or real cameras
- **sensor_msgs/LaserScan**: LIDAR sensor data for obstacle detection
- **nav_msgs/OccupancyGrid**: Map representation for navigation
- **geometry_msgs/PoseStamped**: Goal poses for navigation system
- **geometry_msgs/Twist**: Velocity commands for robot movement
- **tf2_msgs/TFMessage**: Transform information between coordinate frames

### Isaac ROS Bridge Messages
- **isaac_ros_messages/Detection2DArray**: Object detections from Isaac perception modules
- **isaac_ros_messages/TrackedDetections**: Tracked objects with IDs
- **isaac_ros_messages/PointCloudMap**: 3D point cloud representations
- **isaac_ros_messages/IMUMeasurement**: Inertial measurement unit readings

## Perception System Contracts

### VSLAM Module Interface
```
Input: 
- sensor_msgs/Image[] (camera streams)
- sensor_msgs/Imu (inertial measurements)

Output:
- geometry_msgs/PoseStamped (robot pose estimate)
- nav_msgs/OccupancyGrid (environment map)
- visualization_msgs/MarkerArray (landmarks visualization)

Performance Requirements:
- 90% tracking quality in textured environments
- <100ms processing latency for real-time operation
- 85%+ accuracy in environmental mapping tasks (as per SC-002)
```

### Synthetic Data Generation Interface
```
Input:
- IsaacSimulationEnvironment configuration
- Data collection parameters

Output:
- sensor_msgs/Image (synthetic camera data)
- sensor_msgs/LaserScan (synthetic LIDAR data)
- ground truth annotations for training

Performance Requirements:
- Maintain 30 FPS synthetic data generation
- Provide photorealistic rendering quality
- Support domain randomization techniques
```

## Navigation System Contracts

### Nav2 Action Interface
```
Action: nav2_msgs.NavigateToPose
Input:
- geometry_msgs/PoseStamped (goal pose)
- nav2_msgs/BehaviorTree.xml (behavior configuration)

Output:
- nav2_msgs/NavigationResult (success/failure status)
- nav_msgs/Path (executed trajectory)

Performance Requirements:
- 90% success rate in path planning tasks (as per SC-003)
- <5 second re-planning time when obstacles detected
- Collision-free path execution in 95% of scenarios
```

### Global Planner Interface
```
Service: nav_msgs/GetMap
Request:
- None (returns current map)

Response:
- nav_msgs/OccupancyGrid (current static map)
- <timestamp> (when map was last updated)

Performance Requirements:
- Serve map requests in <50ms
- Update map when environment changes detected
- Provide 5cm resolution for humanoid navigation
```

## AI Control System Contracts

### Behavior Selection Interface
```
Input:
- sensor_msgs/Image (perception input)
- nav_msgs/OccupancyGrid (navigation map)
- geometry_msgs/PoseStamped (current robot pose)

Output:
- std_msgs/String (selected behavior name)
- geometry_msgs/Twist (motion command) or 
- action_msgs/GoalInfo (initiate complex behavior)

Performance Requirements:
- Respond to environmental changes within 50ms (as per FR-006 requirements)
- Select appropriate behaviors with 85%+ accuracy
- Handle emergency stops with <10ms response time
```

## Isaac Sim ↔ Isaac ROS Interface

### Simulation Bridge Protocol
```
Publisher: /isaac_sim/bridge_state
Message: std_msgs/String
Content: "simulation_running|paused|stopped"

Subscriber: /isaac_ros/commands
Message: std_msgs/String
Content: "start|pause|reset_simulation"

Performance Requirements:
- <10ms delay between Isaac Sim and Isaac ROS
- Maintain synchronized clock between systems
- Preserve exact timing relationships between components
```

### Sensor Data Bridge
```
Publishers (from Isaac Sim):
- /front_camera/image_raw (sensor_msgs/Image)
- /lidar_scan (sensor_msgs/LaserScan)
- /imu_data (sensor_msgs/Imu)

Subscribers (to Isaac ROS):
- /isaac_ros/perception_pipeline/processed_data
- /isaac_ros/navigation/local_costmap/costmap
- /isaac_ros/control_system/command

Performance Requirements:
- Real-time sensor data transmission (30+ FPS)
- Sub-millisecond synchronization between sensors
- Zero data loss during simulation
```

## Sim-to-Real Transfer Contracts

### Model Compatibility Interface
```
Input Format:
- PyTorch/TensorFlow model file
- Isaac ROS perception node configuration
- Domain randomization parameters

Output Format:
- Optimized model for edge deployment
- Calibration data for real sensors
- Performance characterization report

Success Criteria:
- Maintain 75%+ performance when transferred to real world (as per SC-006)
- Support NVIDIA TensorRT optimization
- Include uncertainty quantification
```

## Error Handling and Recovery

### Perception Failure Modes
- **Lost Tracking**: VSLAM system loses ability to localize
  - Recovery: Reset to known location, reinitialize map
  - Timeout: 30 seconds before manual intervention
  
- **Sensor Malfunction**: Camera or LIDAR data unavailable
  - Recovery: Switch to backup sensors, use dead reckoning
  - Timeout: 5 seconds before navigation pause

### Navigation Failure Modes
- **Path Unreachable**: Global planner cannot find valid path
  - Recovery: Return to previous known location, request new goal
  - Timeout: 10 seconds before failure declaration

- **Obstacle Blocking**: Robot unable to navigate around obstacle
  - Recovery: Rotate in place, request human assistance
  - Timeout: 60 seconds before abandoning goal

### AI Control Failures
- **Invalid Command**: Generated behavior could damage robot
  - Recovery: Trigger emergency stop, switch to safe posture
  - Response time: <10ms

- **Indecision Loop**: AI system unable to select appropriate behavior
  - Recovery: Fall back to safe navigation mode
  - Timeout: 5 seconds before fallback activation

## Quality Assurance Metrics

### System-Level Metrics
- **Perception Accuracy**: Environmental mapping accuracy ≥85% (SC-002)
- **Navigation Success Rate**: Path planning success ≥90% (SC-003)
- **Sim-to-Real Performance**: ≥75% performance maintenance (SC-006)
- **Student Assessment Score**: ≥80% on sim-to-real transfer (SC-005)

### Component-Level Metrics
- **Simulation FPS**: Maintain >30 FPS for interactive rates
- **Control Frequency**: AI control decisions at >20 Hz
- **Communication Latency**: <10ms message transmission between components
- **Map Update Rate**: <1 second update interval when environment changes

These interfaces and contracts serve as the foundation for the AI-Robot Brain module, ensuring consistent and predictable interactions between the various components of the NVIDIA Isaac ecosystem.