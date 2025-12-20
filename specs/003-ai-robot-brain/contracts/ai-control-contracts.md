# AI Control System Contracts: AI-Robot Brain (NVIDIA Isaac)

## Overview
This document defines the contracts and interfaces for the AI control systems in Module 3: The AI-Robot Brain. These contracts cover the integration of perception and navigation systems, intelligent behavior selection, and adaptive responses to environmental conditions using NVIDIA Isaac tools.

## AI Control Interface

### Input Data Streams
```
Topic: /perception/fused_objects
Type: isaac_ros_messages/Detection2DArray
Rate: 10 Hz (minimum)
Purpose: Environmental perception input for decision making

Topic: /navigation/status
Type: std_msgs/String
Rate: 1 Hz (minimum)
Purpose: Navigation system status for behavior planning

Topic: /navigation/current_path
Type: nav_msgs/Path
Rate: 10 Hz (minimum)
Purpose: Current navigation path for coordinated control

Topic: /robot/pose
Type: geometry_msgs/PoseStamped
Rate: 50 Hz (minimum)
Purpose: Current robot pose for AI control decisions

Topic: /robot/joint_states
Type: sensor_msgs/JointState
Rate: 100 Hz (minimum)
Purpose: Robot joint positions for control output validation
```

### AI Control Command Output
```
Topic: /ai_control/command
Type: geometry_msgs/Twist
Rate: 50 Hz (minimum)
Linear Vel: X, Y, Z velocities (m/s)
Angular Vel: Roll, Pitch, Yaw rates (rad/s)
Purpose: Velocity commands from AI control system

Topic: /ai_control/behavior
Type: std_msgs/String
Rate: On behavior change (maximum 10 Hz)
Content: Name of currently selected behavior
Purpose: Behavior selection notification for logging

Topic: /ai_control/behavior_status
Type: std_msgs/String
Rate: 10 Hz (minimum)
Content: "idle|executing|complete|error" for current behavior
Purpose: Behavior execution status
```

## Behavior Library Interface

### Behavior Execution Service
```
Service: /ai_control/execute_behavior
Type: Custom service definition
Request:
  - behavior_name: string (name of behavior to execute)
  - parameters: string (JSON string of behavior parameters)
  - timeout: float (maximum execution time in seconds)

Response:
  - success: bool (whether behavior execution started)
  - execution_id: string (identifier for this execution)
  - message: string (status message)
```

### Behavior Registry Service
```
Service: /ai_control/list_behaviors
Type: std_srvs/Empty
Response:
  - behavior_list: string[] (array of available behavior names)
  - behavior_descriptions: string[] (descriptions for each behavior)
```

## Decision Making Interface

### Perception-Action Mapping
```
Topic: /ai_control/perception_action_map
Type: Custom message type
Rate: 1 Hz (minimum)
Content:
  - environment_state: string (categorized environmental state)
  - suggested_action: string (recommended high-level action)
  - confidence: float (confidence in suggested action, 0.0-1.0)
  - alternatives: string[] (alternative actions with probabilities)
Purpose: Mapping from perception data to action selection
```

### Goal Prioritization Interface
```
Topic: /ai_control/goal_priority
Type: Custom message type
Rate: On goal change (maximum 5 Hz)
Content:
  - current_goal: geometry_msgs/PoseStamped (active navigation goal)
  - goal_priority: int8 (priority level 0-10)
  - goal_type: string (type: exploration, navigation, avoidance, etc.)
  - goal_alternatives: geometry_msgs/PoseStamped[] (alternative goals)
Purpose: Goal selection and prioritization based on perception
```

## Isaac ROS AI Integration

### AI Model Interface
```
Service: /ai_control/load_model
Type: Custom service definition
Request:
  - model_path: string (path to AI model file)
  - model_type: string (type: neural_network, decision_tree, etc.)
  - model_config: string (JSON configuration for the model)
  
Response:
  - success: bool (whether model was loaded successfully)
  - model_id: string (identifier for the loaded model)
  - input_shape: int[] (expected input dimensions)
  - output_shape: int[] (expected output dimensions)
```

### Model Inference Interface
```
Topic: /ai_control/model_input
Type: sensor_msgs/CompressedImage (or appropriate sensor type)
Rate: Variable based on model requirements
Purpose: Input data for AI model inference

Topic: /ai_control/model_output
Type: std_msgs/Float32MultiArray (or appropriate output type)
Rate: Model inference rate
Purpose: AI model predictions for behavior selection
```

## Performance Contracts

### AI Control Performance Requirements
- **Decision Making Frequency**: Respond to environmental changes within 50ms (per FR-006)
- **Behavior Selection Accuracy**: Select appropriate behaviors with 85%+ accuracy
- **Response Time**: React to emergency conditions within 10ms
- **Computational Efficiency**: Use <70% of available GPU resources
- **Memory Usage**: <4GB for full AI control system

### Intelligence Metrics
- **Adaptability**: Adjust behavior appropriately to 90%+ of environmental changes
- **Goal Achievement**: Complete assigned tasks with 85%+ success rate (per SC-004)
- **Safety Compliance**: Never issue commands that violate safety constraints
- **Learning Efficiency**: Improve performance with experience when learning enabled
- **Robustness**: Continue operation during partial system failures

### Integration Performance
- **Perception Integration**: Process perception data with <100ms latency
- **Navigation Integration**: Coordinate with navigation system at >10Hz
- **Control Output**: Generate control commands at >50Hz for smooth operation
- **State Consistency**: Maintain consistent internal state across modules

## Error Handling Contracts

### AI System Failures
- **Model Inference Failure**:
  - Action: Switch to safe default behavior
  - Timeout: 3 seconds before fallback activation
  - Recovery: Attempt model reload or use rule-based system
  
- **Input Data Corruption**:
  - Action: Request fresh data from perception system
  - Timeout: 1 second before treating as sensor failure
  - Recovery: Use backup sensor data or safe mode
  
- **Behavior Execution Failure**:
  - Action: Cancel current behavior, select alternative
  - Timeout: Behavior-specific timeout from service call
  - Recovery: Execute safe stop and request human assistance if needed

### Safety Contracts
- **Safe State Transition**:
  - Trigger: Any safety system activation
  - Action: Immediate transition to safe robot posture
  - Rate: Complete within 100ms of activation
  
- **Constraint Violation**:
  - Trigger: Command that violates physical/joint limits
  - Action: Clip command to limits or issue safe stop
  - Rate: Check all commands in real-time

## Quality Assurance Metrics

### AI Control Quality Metrics
- **Task Completion**: ≥85% of final AI control projects completed (per SC-004)
- **Intelligent Decision Making**: 90%+ accuracy in appropriate behavior selection
- **Adaptive Responses**: Correct response to 90%+ of environmental changes
- **Consistency**: Repeat same scenario with 90%+ consistent behavior selection
- **Efficiency**: Complete tasks with minimal redundant actions

### Integration Quality Metrics
- **Perception Integration**: Successfully process 95%+ of perception inputs
- **Navigation Coordination**: Achieve 95%+ alignment with navigation goals
- **Sensor Fusion**: Appropriately weight inputs from multiple sensors
- **Temporal Consistency**: Maintain proper timing relationships between inputs

### Learning and Adaptation Metrics
- **Performance Improvement**: Show improvement over time when learning enabled
- **Generalization**: Apply learned behaviors to new but similar scenarios
- **Transfer Learning**: Apply sim-trained models to real-world scenarios at ≥75% performance (per SC-006)
- **Uncertainty Handling**: Appropriately respond when confidence is low

## Behavior Library Specifications

### Core Behaviors
- **Safe Navigation**: Navigate to goal while avoiding obstacles safely
- **Exploration**: Systematically explore unknown environment areas
- **Object Approach**: Approach specified objects while maintaining safe distance
- **Human Following**: Track and follow human subjects at safe distance
- **Area Patrol**: Systematically patrol specified area coverage
- **Emergency Stop**: Immediate stop with safe posture transition

### Behavior Contract Template
Each behavior must implement:
- Input validation (check all parameters before execution)
- Safety checks (ensure actions are safe for robot and environment)
- Progress monitoring (track execution progress and detect failures)
- Graceful degradation (handle partial failures appropriately)
- Resource management (monitor and manage computational resources)

These contracts ensure consistent and predictable behavior of the AI control systems in the AI-Robot Brain module, enabling students to build intelligent robotic applications while understanding the performance characteristics and safety requirements of AI-driven robot control.