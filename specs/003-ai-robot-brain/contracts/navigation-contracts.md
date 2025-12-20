# Navigation System Contracts: AI-Robot Brain (NVIDIA Isaac)

## Overview
This document defines the contracts and interfaces for the navigation systems in Module 3: The AI-Robot Brain. These contracts cover Nav2 integration with Isaac ROS, path planning algorithms, obstacle avoidance, and navigation execution using NVIDIA Isaac tools.

## Navigation Action Interface

### Navigation Goal Service
```
Action: nav2_msgs.NavigateToPose
Goal Topic: /navigate_to_pose
Result Topic: /navigate_to_pose/_result
Feedback Topic: /navigate_to_pose/_feedback

Goal Message:
- pose: geometry_msgs/PoseStamped
  - header: std_msgs/Header (with timestamp and frame_id)
  - pose: geometry_msgs/Pose (position and orientation)
- behavior_tree: string (path to behavior tree XML file)

Result Message:
- status: int8 (action status: succeeded, aborted, canceled)
- message: string (description of result)
```

### Cancel Navigation
```
Topic: /navigate_to_pose/_cancel
Type: action_msgs/GoalID
Purpose: Cancel current navigation goal
Rate: Request-response
```

## Path Planning Interfaces

### Global Planner
```
Service: /global_costmap/clear_entirely
Type: std_srvs/Empty
Purpose: Clear global costmap to reset planning
Response: Acknowledgment when cleared

Topic: /global_costmap/costmap
Type: nav_msgs/OccupancyGrid
Rate: 1 Hz (minimum)
Resolution: 5cm per cell
Purpose: Global path planning costmap

Topic: /global_costmap/costmap_updates
Type: map_msgs/OccupancyGridUpdate
Rate: On change
Purpose: Incremental updates to global costmap
```

### Local Planner
```
Service: /local_costmap/clear_entirely
Type: std_srvs/Empty
Purpose: Clear local costmap to reset local planning
Response: Acknowledgment when cleared

Topic: /local_costmap/costmap
Type: nav_msgs/OccupancyGrid
Rate: 5 Hz (minimum)
Resolution: 10cm per cell
Purpose: Local obstacle avoidance planning
```

## Obstacle Detection Interface

### Obstacle Sources
```
Topic: /perception/fused_objects
Type: isaac_ros_messages/Detection2DArray
Rate: 10 Hz (minimum)
Purpose: Dynamic obstacle input for navigation

Topic: /sensors/lidar_scan
Type: sensor_msgs/LaserScan
Rate: 10 Hz (minimum)
Purpose: Static obstacle detection for costmaps

Topic: /perception/environment_map
Type: nav_msgs/OccupancyGrid
Rate: 1 Hz (minimum)
Purpose: High-level environment representation
```

### Collision Avoidance Output
```
Topic: /navigation/velocity_safety_limiter
Type: geometry_msgs/Twist
Rate: 50 Hz (minimum)
Purpose: Velocity commands with safety limits applied
Linear Vel: Constrained to avoid collisions
Angular Vel: Constrained to avoid collisions
```

## Isaac ROS Navigation Integration

### Navigation Status Interface
```
Topic: /navigation/status
Type: std_msgs/String
Rate: 1 Hz (minimum)
Content: "localizing|planning|executing|idle|error"
Purpose: Current navigation system state

Topic: /navigation/current_path
Type: nav_msgs/Path
Rate: 10 Hz (minimum)
Purpose: Currently executing path in robot frame
```

### Behavior Tree Interface
```
Service: /navigation/load_behavior_tree
Type: nav2_msgs/Load bt_xml_path: string (path to behavior tree XML file)
  
Response:
  - success: bool (whether the tree was loaded)
  - message: string (description of result)
```

## Performance Contracts

### Navigation Success Requirements
- **Path Planning Success Rate**: 90%+ success in path planning tasks (SC-003)
- **Collision Avoidance**: 100% success in avoiding static obstacles
- **Dynamic Obstacle Response**: <1 second reaction time to new obstacles
- **Path Following Accuracy**: <10cm lateral error from planned path
- **Goal Achievement**: 95%+ success in reaching designated goals

### Navigation Performance Requirements
- **Global Planning Time**: <5 seconds for 100m path
- **Local Planning Rate**: >10 Hz update rate
- **Recovery Behavior**: Activate within 2 seconds of getting stuck
- **Memory Usage**: <2GB during active navigation
- **CPU Utilization**: <80% sustained during navigation

### Robustness Requirements
- **Map Consistency**: Handle dynamic changes in environment
- **Recovery Success**: 90%+ success rate for recovery behaviors
- **Localization Maintenance**: Maintain position even during navigation
- **Sensor Failure Handling**: Fallback navigation with partial sensor data

## Error Handling Contracts

### Navigation Failures
- **No Valid Path**: 
  - Action: Cancel navigation with descriptive error
  - Timeout: 30 seconds before failure declaration
  - Recovery: Return to safe pose and request human assistance
  
- **Stuck Robot**:
  - Action: Execute recovery behaviors
  - Timeout: 30 seconds of recovery attempts
  - Recovery: Spin, backup, or return to previous safe pose
  
- **Localization Lost**:
  - Action: Pause navigation and attempt relocalization
  - Timeout: 30 seconds to reestablish position
  - Recovery: Return to last known good position

### Safety Contracts
- **Emergency Stop**: 
  - Trigger: Any safety violation
  - Response: Immediate halt within 100ms
  - State: Enter safe mode awaiting manual reset
  
- **Obstacle Proximity**:
  - Trigger: <0.5m to any obstacle
  - Response: Reduce speed proportionally to distance
  - Minimum: Stop if <0.2m to obstacle

## Quality Assurance Metrics

### Navigation Metrics
- **Success Rate**: ≥90% goal achievement (per SC-003)
- **Path Efficiency**: Achieved path ≤ 1.2 × optimal path length
- **Smoothness**: Lateral acceleration < 1.0 m/s²
- **Execution Time**: Reach goal within 1.5 × optimal time
- **Consistency**: Repeat same goal with <5cm end-point variance

### Safety Metrics
- **Collision Rate**: 0% collisions with static/dynamic obstacles
- **Safe Distance**: Maintains >0.3m from obstacles during navigation
- **Emergency Response**: Stop within 0.5m from emergency signal
- **Recovery Success**: 90%+ success rate for recovery behaviors

### Performance Metrics
- **Computational Efficiency**: 
  - Global planning: <5% of available CPU
  - Local planning: <10% of available CPU
  - Memory usage: <2GB during operation

These contracts ensure consistent and predictable behavior of the navigation systems in the AI-Robot Brain module, enabling students to build reliable navigation applications while understanding the performance characteristics and safety requirements of autonomous robot navigation.