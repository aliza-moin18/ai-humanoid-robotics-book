# Perception System Contracts: AI-Robot Brain (NVIDIA Isaac)

## Overview
This document defines the contracts and interfaces for the perception systems in Module 3: The AI-Robot Brain. These contracts cover VSLAM, synthetic data generation, sensor fusion, and other perception capabilities using NVIDIA Isaac tools.

## VSLAM System Interface

### Input Requirements
```
Topic: /front_camera/image_raw
Type: sensor_msgs/Image
Rate: 30 Hz (minimum)
Resolution: 640x480 (minimum)
Encoding: RGB8 or BGR8
Camera Info: sensor_msgs/CameraInfo (intrinsics/extrinsics)

Topic: /imu/data
Type: sensor_msgs/Imu
Rate: 100 Hz (minimum)
Accuracy: <0.1deg orientation accuracy
```

### Output Specifications
```
Topic: /vslam/pose
Type: geometry_msgs/PoseStamped
Frame ID: /map
Rate: 30 Hz (minimum)
Accuracy: <5cm positional, <2deg rotational

Topic: /vslam/map
Type: nav_msgs/OccupancyGrid
Resolution: 5cm per cell (maximum)
Update Rate: 1 Hz (minimum)
Accuracy: 85%+ mapping accuracy (per SC-002)
```

### Service Interface
```
Service: /vslam/reset
Type: std_srvs/Empty
Purpose: Reset VSLAM localization and mapping
Response: Acknowledgment when reset complete
Timeout: 5 seconds maximum
```

## Synthetic Data Generation Interface

### Data Collection Service
```
Service: /synthetic_data/start_collection
Type: custom service definition
Parameters:
  - scene_name: string (name of simulation scene)
  - duration: float (collection duration in seconds)
  - output_path: string (path to save collected data)
  - domain_randomization: bool (enable domain randomization)

Response:
  - success: bool (whether collection started successfully)
  - collection_id: string (identifier for the collection session)
  - estimated_completion: float (estimated time to completion in seconds)
```

### Generated Data Format
```
Images:
  - Format: sensor_msgs/Image
  - Resolution: Configurable (minimum 640x480)
  - Encoding: RGB8
  - Annotations: JSON file with bounding boxes, segmentation masks
  - Calibration: sensor_msgs/CameraInfo for each camera

LIDAR:
  - Format: sensor_msgs/LaserScan or sensor_msgs/PointCloud2
  - Density: Configurable ray count
  - Noise model: Configurable Gaussian noise
  - Ground truth: Position, orientation, and depth information

Ground Truth:
  - Object poses: geometry_msgs/PoseStamped array
  - Scene semantics: Segmentation masks and object labels
  - Camera poses: camera-to-world transformations
```

## Sensor Fusion Interface

### Multi-Sensor Data Input
```
Topic: /sensors/camera_front/image_raw
Type: sensor_msgs/Image
Purpose: Primary visual input

Topic: /sensors/camera_left/image_raw
Type: sensor_msgs/Image
Purpose: Secondary visual input for stereo depth

Topic: /sensors/lidar_scan
Type: sensor_msgs/LaserScan
Purpose: Primary depth and obstacle detection

Topic: /sensors/imu
Type: sensor_msgs/Imu
Purpose: Inertial measurements for motion compensation
```

### Fused Perception Output
```
Topic: /perception/fused_objects
Type: isaac_ros_messages/Detection2DArray
Rate: 10 Hz (minimum)
Content:
  - Object bounding boxes with confidence scores
  - 3D position estimates when possible
  - Object classification labels
  - Tracking IDs for multi-frame consistency

Topic: /perception/environment_map
Type: nav_msgs/OccupancyGrid
Resolution: 10cm per cell
Update Rate: 1 Hz (minimum)
Content: Fused representation of static and dynamic obstacles
```

## Performance Contracts

### VSLAM Performance Requirements
- **Localization Accuracy**: <5cm positional error in textured environments
- **Mapping Accuracy**: 85%+ accuracy in environmental mapping tasks (SC-002)
- **Tracking Quality**: 90%+ tracking success rate in well-textured scenes
- **Processing Latency**: <100ms per frame for real-time operation
- **Robustness**: Maintain tracking during 5-second textureless motion

### Synthetic Data Performance Requirements
- **Generation Rate**: 30 FPS synthetic data generation
- **Realism**: Passes human perceptual similarity tests
- **Diversity**: Minimum 1000 unique scene variations
- **Annotation Quality**: >95% accuracy in synthetic labels
- **Domain Randomization**: Configurable parameters for domain adaptation

### Sensor Fusion Performance Requirements
- **Data Synchronization**: <50ms delay between sensor streams
- **Object Detection Accuracy**: 90%+ mAP (mean Average Precision)
- **Detection Range**: 20m for objects >1m^2 at >0.7 confidence
- **False Positive Rate**: <5% in static environments
- **Multi-Object Tracking**: 95%+ ID consistency over 100 frames

## Error Handling Contracts

### Perception System Failures
- **Camera Failure**: Fallback to LIDAR-only tracking
- **IMU Failure**: Continue VSLAM with visual-only odometry
- **LIDAR Failure**: Rely on visual and inertial sensors
- **Processing Overload**: Reduce frame rate or resolution adaptively
- **Map Drift**: Detect and trigger relocalization

### Recovery Procedures
- **Automatic Recovery**: Attempt standard recovery within 5 seconds
- **Manual Intervention**: Signal for human assistance if automatic fails
- **Safe State**: Transition to safe pose when perception unavailable
- **Logging**: Record all failures with full state information

## Quality Assurance Metrics

### Validation Criteria
- **Accuracy Metrics**: 
  - Euclidean distance error < 5cm for pose estimates
  - IoU > 0.7 for object detections
  - ATE (Absolute Trajectory Error) < 1m
- **Performance Metrics**:
  - Processing rate > 10Hz
  - Memory usage < 4GB for full pipeline
  - GPU utilization < 90% sustained
- **Reliability Metrics**:
  - MTBF (Mean Time Between Failures) > 2 hours
  - Recovery success rate > 95%

These contracts ensure consistent and predictable behavior of the perception systems in the AI-Robot Brain module, enabling students to build robust applications while understanding the performance characteristics and limitations of each component.