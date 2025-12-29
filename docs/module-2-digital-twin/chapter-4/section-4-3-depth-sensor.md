---
sidebar_position: 3
---

# Section 4.3: Depth Sensor Integration

## Overview

This section covers the integration and configuration of depth sensors in Gazebo for digital twin applications. Students will learn to set up RGB-D sensors, configure their parameters, and process depth data for 3D perception in humanoid robot simulation environments.

## Depth Sensor Fundamentals in Simulation

### How Depth Sensors Work in Gazebo

Depth sensors in Gazebo simulate RGB-D cameras that provide both color images and depth information. The sensor works by:

1. **Color Capture**: Rendering RGB images of the scene
2. **Depth Calculation**: Computing distance to objects for each pixel
3. **Data Generation**: Creating synchronized RGB and depth images
4. **Noise Application**: Adding realistic sensor noise

### Types of Depth Sensors

- **Stereo Cameras**: Two cameras for depth estimation
- **Structured Light**: Project pattern for depth calculation
- **Time-of-Flight**: Measure light travel time for depth
- **Simulated RGB-D**: Direct depth rendering (Gazebo's approach)

## Basic Depth Sensor Configuration

### RGB-D Camera Example

```xml
<robot name="rgbd_robot">
  <link name="camera_link">
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
    </collision>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0.02" rpy="0 0 0"/>
  </joint>

  <gazebo reference="camera_link">
    <sensor name="rgbd_camera" type="depth">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>30.0</updateRate>
        <cameraName>rgbd_camera</cameraName>
        <imageTopicName>/rgb/image_raw</imageTopicName>
        <depthImageTopicName>/depth/image_raw</depthImageTopicName>
        <pointCloudTopicName>/depth/points</pointCloudTopicName>
        <cameraInfoTopicName>/rgb/camera_info</cameraInfoTopicName>
        <depthImageCameraInfoTopicName>/depth/camera_info</depthImageCameraInfoTopicName>
        <frameName>camera_link</frameName>
        <baseline>0.1</baseline>
        <distortion_k1>0.0</distortion_k1>
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
        <pointCloudCutoff>0.1</pointCloudCutoff>
        <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
        <CxPrime>0</CxPrime>
        <Cx>320.5</Cx>
        <Cy>240.5</Cy>
        <focalLength>320.0</focalLength>
        <hackBaseline>0.07</hackBaseline>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

### Alternative Depth Sensor Configuration

```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    
    <camera name="depth_view">
      <horizontal_fov>1.047197</horizontal_fov>  <!-- 60 degrees in radians -->
      <image>
        <width>320</width>
        <height>240</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>8.0</far>
      </clip>
      <save enabled="false"/>
    </camera>
    
    <plugin name="depth_camera_plugin" filename="libgazebo_ros_depth_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <cameraName>depth_camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <depthImageTopicName>depth/image_raw</depthImageTopicName>
      <pointCloudTopicName>depth/points</pointCloudTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
      <baseline>0.1</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      <pointCloudCutoff>0.1</pointCloudCutoff>
      <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
    </plugin>
  </sensor>
</gazebo>
```

## Depth Sensor Parameters Explained

### Camera Parameters

#### Field of View
- `horizontal_fov`: Horizontal field of view in radians
- Affects how much of the scene is captured

#### Image Resolution
- `width`, `height`: Image dimensions in pixels
- Higher resolution = more detailed images but more computation

#### Clipping Planes
- `near`: Minimum distance for depth measurement (meters)
- `far`: Maximum distance for depth measurement (meters)

### Noise Parameters

Adding realistic noise to depth measurements:

```xml
<camera>
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.007</stddev>  <!-- 7mm standard deviation -->
  </noise>
</camera>
```

### Performance Parameters

- `update_rate`: How often sensor publishes data (Hz)
- `always_on`: Whether sensor runs continuously

## Advanced Depth Sensor Configuration

### Multiple Depth Sensors

For extended coverage or redundancy:

```xml
<!-- Front depth camera -->
<gazebo reference="front_camera_link">
  <sensor name="front_depth" type="depth">
    <!-- Configuration -->
  </sensor>
</gazebo>

<!-- Left depth camera -->
<gazebo reference="left_camera_link">
  <sensor name="left_depth" type="depth">
    <!-- Configuration -->
  </sensor>
</gazebo>

<!-- Right depth camera -->
<gazebo reference="right_camera_link">
  <sensor name="right_depth" type="depth">
    <!-- Configuration -->
  </sensor>
</gazebo>
```

### High-Resolution Configuration

For detailed 3D perception:

```xml
<sensor name="high_res_depth" type="depth">
  <always_on>true</always_on>
  <update_rate>15</update_rate>  <!-- Lower rate to handle higher resolution -->
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>1280</width>  <!-- High resolution -->
      <height>960</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.05</near>   <!-- Closer minimum range -->
      <far>15.0</far>     <!-- Extended range -->
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.005</stddev>  <!-- Lower noise for high-quality sensor -->
    </noise>
  </camera>
</sensor>
```

## Humanoid Robot Depth Sensor Integration

### Head-Mounted RGB-D Camera

For humanoid robots, depth sensors are typically mounted on the head:

```xml
<robot name="humanoid_with_depth_camera">
  <!-- Head link -->
  <link name="head">
    <inertial>
      <mass value="3"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Depth camera link -->
  <link name="depth_camera_link">
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.06 0.08 0.03"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.06 0.08 0.03"/>
      </geometry>
    </collision>
  </link>

  <joint name="depth_camera_joint" type="fixed">
    <parent link="head"/>
    <child link="depth_camera_link"/>
    <origin xyz="0.08 0 0.02" rpy="0 0 0"/>  <!-- Positioned at front of head -->
  </joint>

  <!-- Depth camera configuration -->
  <gazebo reference="depth_camera_link">
    <sensor name="head_depth_camera" type="depth">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>8.0</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </camera>
      <plugin name="head_depth_camera_controller" filename="libgazebo_ros_openni_kinect.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>30.0</updateRate>
        <cameraName>head_depth</cameraName>
        <imageTopicName>/head_camera/rgb/image_raw</imageTopicName>
        <depthImageTopicName>/head_camera/depth/image_raw</depthImageTopicName>
        <pointCloudTopicName>/head_camera/depth/points</pointCloudTopicName>
        <cameraInfoTopicName>/head_camera/rgb/camera_info</cameraInfoTopicName>
        <depthImageCameraInfoTopicName>/head_camera/depth/camera_info</depthImageCameraInfoTopicName>
        <frameName>depth_camera_link</frameName>
        <pointCloudCutoff>0.1</pointCloudCutoff>
        <pointCloudCutoffMax>5.0</pointCloudCutoffMax>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

### Stereo Camera Setup

For more accurate depth estimation:

```xml
<!-- Left camera -->
<link name="stereo_left_camera_link">
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="stereo_left_joint" type="fixed">
  <parent link="head"/>
  <child link="stereo_left_camera_link"/>
  <origin xyz="0.08 -0.03 0.02" rpy="0 0 0"/>
</joint>

<!-- Right camera -->
<link name="stereo_right_camera_link">
  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="stereo_right_joint" type="fixed">
  <parent link="head"/>
  <child link="stereo_right_camera_link"/>
  <origin xyz="0.08 0.03 0.02" rpy="0 0 0"/>
</joint>

<!-- Configure both cameras -->
<gazebo reference="stereo_left_camera_link">
  <sensor name="stereo_left" type="camera">
    <!-- Camera configuration -->
  </sensor>
</gazebo>

<gazebo reference="stereo_right_camera_link">
  <sensor name="stereo_right" type="camera">
    <!-- Camera configuration -->
  </sensor>
</gazebo>
```

## Depth Data Processing

### ROS Message Types

Depth sensors publish several message types:
- `sensor_msgs/Image`: RGB image data
- `sensor_msgs/Image`: Depth image data
- `sensor_msgs/PointCloud2`: 3D point cloud
- `sensor_msgs/CameraInfo`: Camera calibration parameters

### Sample Processing Node

```cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class DepthProcessor {
public:
    DepthProcessor() {
        depth_sub_ = nh_.subscribe("/head_camera/depth/image_raw", 1, 
                                  &DepthProcessor::depthCallback, this);
        rgb_sub_ = nh_.subscribe("/head_camera/rgb/image_raw", 1, 
                                &DepthProcessor::rgbCallback, this);
        obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
    }

private:
    void depthCallback(const sensor_msgs::Image::ConstPtr& depth_msg) {
        try {
            cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_msg, 
                                                                 sensor_msgs::image_encodings::TYPE_32FC1);
            
            // Process depth image to detect obstacles
            cv::Mat depth_image = cv_depth->image;
            cv::Mat obstacle_mask = cv::Mat::zeros(depth_image.size(), CV_8UC1);
            
            // Detect obstacles within 1 meter
            for (int i = 0; i < depth_image.rows; i++) {
                for (int j = 0; j < depth_image.cols; j++) {
                    float depth_value = depth_image.at<float>(i, j);
                    if (depth_value > 0.1 && depth_value < 1.0) {  // Obstacle within 1m
                        obstacle_mask.at<uchar>(i, j) = 255;
                    }
                }
            }
            
            // Convert to point cloud
            sensor_msgs::PointCloud2 pc_msg;
            createPointCloudFromDepth(depth_image, pc_msg);
            obstacle_pub_.publish(pc_msg);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void rgbCallback(const sensor_msgs::Image::ConstPtr& rgb_msg) {
        try {
            cv_bridge::CvImagePtr cv_rgb = cv_bridge::toCvCopy(rgb_msg, 
                                                              sensor_msgs::image_encodings::BGR8);
            // Process RGB image if needed
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void createPointCloudFromDepth(const cv::Mat& depth_image, sensor_msgs::PointCloud2& pc_msg) {
        // Implementation to convert depth image to point cloud
        // This is a simplified example
    }

    ros::NodeHandle nh_;
    ros::Subscriber depth_sub_;
    ros::Subscriber rgb_sub_;
    ros::Publisher obstacle_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_processor");
    DepthProcessor processor;
    ros::spin();
    return 0;
}
```

## Validation and Testing

### Depth Sensor Validation Tests

#### Range Test
```xml
<!-- Test with known distances -->
<model name="depth_test">
  <pose>0 0 0 0 0 0</pose>
  <link name="sensor">
    <inertial>
      <mass>0.1</mass>
    </inertial>
  </link>
  
  <!-- Depth sensor -->
  <gazebo reference="sensor">
    <sensor name="test_depth" type="depth">
      <!-- Configuration -->
    </sensor>
  </gazebo>
  
  <!-- Known objects at specific distances -->
  <model name="wall_1m" static="true">
    <pose>1 0 0 0 0 0</pose>
    <link name="link">
      <collision><geometry><box><size>2 2 0.1</size></box></geometry></collision>
    </link>
  </model>
</model>
```

#### Resolution Test
- Verify that small objects can be detected at various distances
- Test with objects of different sizes

### Point Cloud Quality Assessment

```cpp
// Check point cloud density and completeness
void checkPointCloudQuality(const sensor_msgs::PointCloud2::ConstPtr& pc_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc_msg, *cloud);
    
    ROS_INFO("Point cloud has %d points", (int)cloud->size());
    
    // Check for completeness
    if (cloud->size() < 1000) {
        ROS_WARN("Point cloud is sparse, may need to adjust sensor parameters");
    }
}
```

## Performance Considerations

### Computational Impact

Depth sensors can be computationally intensive:

- **Resolution**: Higher resolution = more computation
- **Update Rate**: Higher rate = more processing
- **Point Cloud Generation**: Can be expensive for high-resolution sensors

### Optimization Techniques

#### Resolution Reduction
```xml
<camera>
  <image>
    <width>320</width>   <!-- Lower resolution -->
    <height>240</height>
    <format>R8G8B8</format>
  </image>
</camera>
```

#### Update Rate Adjustment
```xml
<sensor name="performance_depth" type="depth">
  <update_rate>15</update_rate>  <!-- Lower rate for performance -->
  <!-- Other parameters -->
</sensor>
```

#### Point Cloud Filtering
```cpp
// Only process points within a certain range
void filterPointCloud(const sensor_msgs::PointCloud2::ConstPtr& input, 
                     sensor_msgs::PointCloud2& output, 
                     float min_range, float max_range) {
    // Implementation to filter point cloud by range
}
```

## Troubleshooting Common Issues

### Issue 1: Depth Sensor Not Publishing Data
- **Check**: Sensor plugin configuration
- **Verify**: Topic remapping
- **Test**: Use `rostopic echo` to verify data

### Issue 2: Incorrect Depth Values
- **Check**: Clipping plane configuration
- **Verify**: Coordinate frame alignment
- **Adjust**: Camera parameters

### Issue 3: Performance Issues
- **Reduce**: Image resolution
- **Lower**: Update rate
- **Optimize**: Point cloud processing

## Best Practices

1. **Realistic Parameters**: Use parameters that match real depth sensors
2. **Appropriate Resolution**: Balance accuracy with performance
3. **Mounting Position**: Place sensors in realistic positions on the robot
4. **Validation Testing**: Test with known objects at known distances
5. **Noise Modeling**: Include realistic noise in simulation
6. **Multiple Sensors**: Consider redundancy for critical applications
7. **Point Cloud Processing**: Optimize for computational efficiency
8. **Calibration**: Ensure proper camera calibration parameters

## Next Steps

After learning about depth sensor integration, proceed to [Section 4.4: Sensor Data Processing and Visualization](../chapter-4/section-4-4-sensor-data-processing) to learn about processing and visualizing sensor data in digital twin simulations.