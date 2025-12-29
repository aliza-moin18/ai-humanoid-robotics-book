---
sidebar_position: 4
---

# Section 4.4: Sensor Data Processing and Visualization

## Overview

This section covers the processing and visualization of sensor data in digital twin simulations. Students will learn to integrate data from multiple sensors, implement sensor fusion techniques, and visualize sensor data in real-time for humanoid robot perception and monitoring.

## Sensor Data Processing Pipeline

### Overview of Processing Pipeline

The sensor data processing pipeline typically includes:

1. **Data Acquisition**: Receiving raw sensor measurements
2. **Preprocessing**: Filtering and calibration of raw data
3. **Fusion**: Combining data from multiple sensors
4. **Interpretation**: Extracting meaningful information
5. **Visualization**: Presenting processed data to users

### Basic Processing Node Structure

```cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class SensorProcessor {
public:
    SensorProcessor() {
        // Initialize subscribers for different sensor types
        lidar_sub_ = nh_.subscribe("/scan", 1, &SensorProcessor::lidarCallback, this);
        imu_sub_ = nh_.subscribe("/imu/data", 1, &SensorProcessor::imuCallback, this);
        depth_sub_ = nh_.subscribe("/head_camera/depth/image_raw", 1, &SensorProcessor::depthCallback, this);
        
        // Publishers for processed data
        obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacles", 1);
        pose_pub_ = nh_.advertise<geometry_msgs::Pose>("estimated_pose", 1);
    }

private:
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // Process LiDAR data
        processLidarData(*scan);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
        // Process IMU data
        processImuData(*imu);
    }

    void depthCallback(const sensor_msgs::Image::ConstPtr& depth) {
        // Process depth data
        processDepthData(*depth);
    }

    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber depth_sub_;
    ros::Publisher obstacle_pub_;
    ros::Publisher pose_pub_;
};
```

## Multi-Sensor Data Fusion

### Kalman Filter Integration

Combining IMU and other sensor data:

```cpp
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class SensorFusion {
public:
    SensorFusion() : 
        nh_("~"),
        pose_estimate_(geometry_msgs::PoseWithCovarianceStamped()),
        initialized_(false) {
        
        imu_sub_ = nh_.subscribe("imu/data", 1, &SensorFusion::imuCallback, this);
        encoder_sub_ = nh_.subscribe("encoder/pose", 1, &SensorFusion::encoderCallback, this);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("fused_pose", 1);
        
        // Initialize covariance matrix
        for (int i = 0; i < 36; i++) {
            pose_estimate_.pose.covariance[i] = 0.0;
        }
    }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        if (!initialized_) {
            // Initialize with first IMU reading
            initializeFilter(*imu_msg);
            return;
        }

        // Update filter with IMU data
        updateWithImu(*imu_msg);
        publishEstimate();
    }

    void encoderCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& encoder_msg) {
        if (!initialized_) {
            return;
        }

        // Update filter with encoder data
        updateWithEncoder(*encoder_msg);
        publishEstimate();
    }

    void initializeFilter(const sensor_msgs::Imu& imu) {
        // Initialize pose estimate with IMU orientation
        pose_estimate_.pose.pose.orientation = imu.orientation;
        initialized_ = true;
    }

    void updateWithImu(const sensor_msgs::Imu& imu) {
        // Apply Kalman filter update using IMU data
        // Simplified example - real implementation would be more complex
    }

    void updateWithEncoder(const geometry_msgs::PoseWithCovarianceStamped& encoder) {
        // Apply Kalman filter update using encoder data
        // Simplified example - real implementation would be more complex
    }

    void publishEstimate() {
        pose_estimate_.header.stamp = ros::Time::now();
        pose_estimate_.header.frame_id = "map";
        pose_pub_.publish(pose_estimate_);
    }

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber encoder_sub_;
    ros::Publisher pose_pub_;
    geometry_msgs::PoseWithCovarianceStamped pose_estimate_;
    bool initialized_;
};
```

### Sensor Fusion with Different Rates

Handling sensors with different update rates:

```cpp
class MultiRateFusion {
public:
    MultiRateFusion() {
        // High-rate IMU (200Hz)
        imu_sub_ = nh_.subscribe("/imu/data", 10, &MultiRateFusion::imuCallback, this);
        
        // Medium-rate LiDAR (10Hz)
        lidar_sub_ = nh_.subscribe("/scan", 1, &MultiRateFusion::lidarCallback, this);
        
        // Low-rate GPS (1Hz) - if available in simulation
        gps_sub_ = nh_.subscribe("/gps/fix", 1, &MultiRateFusion::gpsCallback, this);
        
        pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("fused_pose", 1);
    }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
        // Store IMU data and update pose estimate
        latest_imu_ = *imu;
        predictPose();
    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // Update pose estimate with LiDAR data
        updateWithLidar(*scan);
        publishPose();
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps) {
        // Update pose estimate with GPS data
        updateWithGps(*gps);
        publishPose();
    }

    void predictPose() {
        // Use IMU data to predict pose between corrections
    }

    void updateWithLidar(const sensor_msgs::LaserScan& scan) {
        // Use LiDAR data to correct pose estimate
    }

    void updateWithGps(const sensor_msgs::NavSatFix& gps) {
        // Use GPS data to correct pose estimate
    }

    void publishPose() {
        // Publish the fused pose estimate
    }

    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber gps_sub_;
    ros::Publisher pose_pub_;
    sensor_msgs::Imu latest_imu_;
};
```

## Sensor Data Visualization

### RViz Configuration

Creating an RViz configuration for sensor visualization:

```yaml
Panels:
  - Class: rviz/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /Grid1
        - /LaserScan1
        - /PointCloud21
        - /Image1
        - /TF1
      Splitter Ratio: 0.5
    Tree Height: 775
  - Class: rviz/Selection
    Name: Selection
  - Class: rviz/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: LaserScan

Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic: /scan
      Unreliable: false
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/PointCloud2
      Color: 255; 255; 255
      Color Transformer: RGB8
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: PointCloud2
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic: /head_camera/depth/points
      Unreliable: false
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Class: rviz/Image
      Enabled: true
      Image Topic: /head_camera/rgb/image_raw
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Image
      Normalize Range: true
      Queue Size: 2
      Transport Hint: raw
      Unreliable: false
      Value: true
    - Class: rviz/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        {}
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
    - Class: rviz/Measure
    - Class: rviz/SetInitialPose
      Topic: /initialpose
    - Class: rviz/SetGoal
      Topic: /move_base_simple/goal
    - Class: rviz/PublishPoint
      Single click: true
      Topic: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.5
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1056
  Hide Left Dock: false
  Hide Right Dock: false
  Image:
    collapsed: false
  QMainWindow State: 000000ff00000000fd00000004000000000000015600000396fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d00000396000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f00000396fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d00000396000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e100000197000000030000073f0000003efc0100000002fb0000000800540069006d006501000000000000073f000002eb00fffffffb0000000800540069006d00650100000000000004500000000000000000000005cf0000039600000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Width: 1855
  X: 65
  Y: 24
```

### Custom Visualization Tools

Creating a custom visualization node:

```cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

class SensorVisualizer {
public:
    SensorVisualizer() {
        lidar_sub_ = nh_.subscribe("/scan", 1, &SensorVisualizer::lidarCallback, this);
        imu_sub_ = nh_.subscribe("/imu/data", 1, &SensorVisualizer::imuCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("sensor_visualization", 1);
    }

private:
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // Create visualization marker for LiDAR data
        visualization_msgs::Marker points;
        points.header.frame_id = scan->header.frame_id;
        points.header.stamp = scan->header.stamp;
        points.ns = "lidar_points";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.05;
        points.scale.y = 0.05;
        points.color.r = 1.0f;
        points.color.g = 0.0f;
        points.color.b = 0.0f;
        points.color.a = 1.0;

        // Convert scan data to points
        float angle = scan->angle_min;
        for (auto range : scan->ranges) {
            if (range > scan->range_min && range < scan->range_max) {
                geometry_msgs::Point p;
                p.x = range * cos(angle);
                p.y = range * sin(angle);
                p.z = 0;
                points.points.push_back(p);
            }
            angle += scan->angle_increment;
        }

        marker_pub_.publish(points);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
        // Create visualization marker for IMU orientation
        visualization_msgs::Marker orientation_arrow;
        orientation_arrow.header.frame_id = imu->header.frame_id;
        orientation_arrow.header.stamp = imu->header.stamp;
        orientation_arrow.ns = "imu_orientation";
        orientation_arrow.action = visualization_msgs::Marker::ADD;
        orientation_arrow.pose.orientation = imu->orientation;
        orientation_arrow.id = 1;
        orientation_arrow.type = visualization_msgs::Marker::ARROW;
        orientation_arrow.scale.x = 0.5;  // Length of arrow
        orientation_arrow.scale.y = 0.1;  // Width of arrow
        orientation_arrow.scale.z = 0.1;  // Height of arrow
        orientation_arrow.color.r = 0.0f;
        orientation_arrow.color.g = 1.0f;
        orientation_arrow.color.b = 0.0f;
        orientation_arrow.color.a = 1.0;

        marker_pub_.publish(orientation_arrow);
    }

    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher marker_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_visualizer");
    SensorVisualizer visualizer;
    ros::spin();
    return 0;
}
```

## Real-Time Processing Techniques

### Threading for Performance

Using multiple threads for sensor processing:

```cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/thread.hpp>

class ThreadingSensorProcessor {
public:
    ThreadingSensorProcessor() {
        lidar_sub_ = nh_.subscribe("/scan", 1, &ThreadingSensorProcessor::lidarCallback, this);
        imu_sub_ = nh_.subscribe("/imu/data", 1, &ThreadingSensorProcessor::imuCallback, this);
        
        // Start processing threads
        lidar_thread_ = boost::thread(boost::bind(&ThreadingSensorProcessor::processLidarQueue, this));
        imu_thread_ = boost::thread(boost::bind(&ThreadingSensorProcessor::processImuQueue, this));
    }

    ~ThreadingSensorProcessor() {
        // Stop threads
        lidar_thread_.interrupt();
        imu_thread_.interrupt();
        lidar_thread_.join();
        imu_thread_.join();
    }

private:
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        boost::mutex::scoped_lock lock(lidar_mutex_);
        lidar_queue_.push_back(*scan);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
        boost::mutex::scoped_lock lock(imu_mutex_);
        imu_queue_.push_back(*imu);
    }

    void processLidarQueue() {
        while (ros::ok()) {
            if (!lidar_queue_.empty()) {
                boost::mutex::scoped_lock lock(lidar_mutex_);
                sensor_msgs::LaserScan scan = lidar_queue_.front();
                lidar_queue_.pop_front();
                lock.unlock();
                
                // Process the scan
                processLidarData(scan);
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
    }

    void processImuQueue() {
        while (ros::ok()) {
            if (!imu_queue_.empty()) {
                boost::mutex::scoped_lock lock(imu_mutex_);
                sensor_msgs::Imu imu = imu_queue_.front();
                imu_queue_.pop_front();
                lock.unlock();
                
                // Process the IMU data
                processImuData(imu);
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
    }

    void processLidarData(const sensor_msgs::LaserScan& scan) {
        // Implementation for LiDAR processing
    }

    void processImuData(const sensor_msgs::Imu& imu) {
        // Implementation for IMU processing
    }

    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    
    boost::thread lidar_thread_;
    boost::thread imu_thread_;
    
    boost::mutex lidar_mutex_;
    boost::mutex imu_mutex_;
    
    std::deque<sensor_msgs::LaserScan> lidar_queue_;
    std::deque<sensor_msgs::Imu> imu_queue_;
};
```

### Asynchronous Processing

Using asynchronous callbacks for better performance:

```cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

class AsyncSensorProcessor {
public:
    AsyncSensorProcessor() : 
        io_service_(),
        work_(io_service_),
        processing_thread_(boost::bind(&boost::asio::io_service::run, &io_service_)) {
        
        lidar_sub_ = nh_.subscribe("/scan", 1, &AsyncSensorProcessor::lidarCallback, this);
        processed_pub_ = nh_.advertise<std_msgs::Float32>("processing_time", 1);
    }

    ~AsyncSensorProcessor() {
        io_service_.stop();
        processing_thread_.join();
    }

private:
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // Post the processing task to the async service
        io_service_.post(boost::bind(&AsyncSensorProcessor::processLidarAsync, 
                                    this, *scan));
    }

    void processLidarAsync(const sensor_msgs::LaserScan& scan) {
        ros::Time start = ros::Time::now();
        
        // Perform the actual processing
        float obstacle_distance = findNearestObstacle(scan);
        
        ros::Time end = ros::Time::now();
        float processing_time = (end - start).toSec();
        
        // Publish processing time
        std_msgs::Float32 time_msg;
        time_msg.data = processing_time;
        processed_pub_.publish(time_msg);
    }

    float findNearestObstacle(const sensor_msgs::LaserScan& scan) {
        float min_range = std::numeric_limits<float>::max();
        for (auto range : scan.ranges) {
            if (range > scan.range_min && range < scan.range_max && range < min_range) {
                min_range = range;
            }
        }
        return min_range;
    }

    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Publisher processed_pub_;
    
    boost::asio::io_service io_service_;
    boost::asio::io_service::work work_;
    boost::thread processing_thread_;
};
```

## Data Filtering and Preprocessing

### Noise Reduction

Applying filters to sensor data:

```cpp
#include <vector>
#include <algorithm>

class SensorFilter {
public:
    SensorFilter(int window_size = 5) : window_size_(window_size) {}

    // Moving average filter
    double movingAverageFilter(double new_value) {
        values_.push_back(new_value);
        if (values_.size() > window_size_) {
            values_.erase(values_.begin());
        }

        double sum = 0;
        for (auto val : values_) {
            sum += val;
        }
        return sum / values_.size();
    }

    // Median filter
    double medianFilter(double new_value) {
        values_.push_back(new_value);
        if (values_.size() > window_size_) {
            values_.erase(values_.begin());
        }

        std::vector<double> sorted_values = values_;
        std::sort(sorted_values.begin(), sorted_values.end());
        
        int n = sorted_values.size();
        if (n % 2 == 0) {
            return (sorted_values[n/2 - 1] + sorted_values[n/2]) / 2.0;
        } else {
            return sorted_values[n/2];
        }
    }

private:
    int window_size_;
    std::vector<double> values_;
};

// Using the filter with sensor data
class FilteredSensorProcessor {
public:
    FilteredSensorProcessor() : 
        range_filter_(7),  // 7-point moving average
        angle_filter_(5) {  // 5-point median filter
        
        lidar_sub_ = nh_.subscribe("/scan", 1, &FilteredSensorProcessor::lidarCallback, this);
        filtered_pub_ = nh_.advertise<sensor_msgs::LaserScan>("filtered_scan", 1);
    }

private:
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        sensor_msgs::LaserScan filtered_scan = *scan;
        
        // Apply filtering to each range reading
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            if (scan->ranges[i] > scan->range_min && scan->ranges[i] < scan->range_max) {
                filtered_scan.ranges[i] = range_filter_.movingAverageFilter(scan->ranges[i]);
            }
        }
        
        filtered_pub_.publish(filtered_scan);
    }

    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Publisher filtered_pub_;
    SensorFilter range_filter_;
    SensorFilter angle_filter_;
};
```

## Performance Monitoring

### Processing Time Analysis

Monitoring performance of sensor processing:

```cpp
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>

class PerformanceMonitor {
public:
    PerformanceMonitor() {
        lidar_sub_ = nh_.subscribe("/scan", 1, &PerformanceMonitor::lidarCallback, this);
        timing_pub_ = nh_.advertise<std_msgs::Float32>("processing_time", 1);
        rate_pub_ = nh_.advertise<std_msgs::Float32>("processing_rate", 1);
    }

private:
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        ros::Time start = ros::Time::now();
        
        // Process the sensor data
        processScan(*scan);
        
        ros::Time end = ros::Time::now();
        float processing_time = (end - start).toSec();
        
        // Publish processing time
        std_msgs::Float32 time_msg;
        time_msg.data = processing_time;
        timing_pub_.publish(time_msg);
        
        // Calculate and publish processing rate
        static ros::Time last_time = ros::Time::now();
        float rate = 1.0 / (end - last_time).toSec();
        last_time = end;
        
        std_msgs::Float32 rate_msg;
        rate_msg.data = rate;
        rate_pub_.publish(rate_msg);
    }

    void processScan(const sensor_msgs::LaserScan& scan) {
        // Implementation of scan processing
        // This is where you would implement your actual processing algorithm
    }

    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Publisher timing_pub_;
    ros::Publisher rate_pub_;
};
```

## Best Practices for Sensor Data Processing

1. **Efficient Data Structures**: Use appropriate data structures for your processing needs
2. **Threading**: Separate sensor acquisition from processing when possible
3. **Filtering**: Apply appropriate filters to reduce noise in sensor data
4. **Validation**: Always validate sensor data before using it in control algorithms
5. **Visualization**: Provide visualization tools to debug sensor processing
6. **Performance Monitoring**: Monitor processing times to ensure real-time performance
7. **Modular Design**: Keep sensor processing modules separate and reusable
8. **Calibration**: Account for sensor calibration parameters in processing

## Troubleshooting Common Issues

### Issue 1: High Processing Latency
- **Check**: Threading implementation
- **Verify**: Computational complexity of algorithms
- **Optimize**: Use more efficient algorithms or reduce data resolution

### Issue 2: Data Synchronization Problems
- **Check**: Time stamps on sensor messages
- **Verify**: Use message_filters for synchronization
- **Adjust**: Buffer sizes for message synchronization

### Issue 3: Memory Issues with Large Data Sets
- **Check**: Memory usage of point cloud processing
- **Verify**: Proper cleanup of processed data
- **Optimize**: Use streaming processing instead of buffering

## Next Steps

After mastering sensor data processing and visualization, proceed to [Chapter 5: Unity Visualization and Human-Robot Interaction](../chapter-5/index) to learn about visualizing sensor data and implementing human-robot interaction in Unity.