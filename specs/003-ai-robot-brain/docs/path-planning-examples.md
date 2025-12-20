# Path Planning Examples Using Isaac Tools

## Overview

This document provides practical examples of path planning implementations using Isaac tools, including Nav2 for path planning and Isaac ROS for perception. The examples demonstrate how to integrate different path planning algorithms with Isaac's simulation and perception capabilities.

## Example 1: Basic Path Planning with Navfn (Dijkstra-based)

This example demonstrates a simple path planning setup using the Navfn planner, which implements a Dijkstra-like algorithm.

### Configuration

```yaml
# basic_path_planning.yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5       # Acceptable distance to goal
      use_astar: false     # Use Dijkstra instead of A*
      allow_unknown: true  # Plan through unknown space
```

### Implementation Code

```cpp
// basic_path_planning.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>

class BasicPathPlanner : public rclcpp::Node
{
public:
    BasicPathPlanner() : Node("basic_path_planner")
    {
        // Initialize the transform buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create action client for path planning
        this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
            this, "compute_path_to_pose");
            
        RCLCPP_INFO(this->get_logger(), "Basic Path Planner initialized");
    }

    void planPath(double start_x, double start_y, double goal_x, double goal_y)
    {
        // Wait for the action server to be ready
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        // Create the goal message
        auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = goal_x;
        goal_msg.pose.pose.position.y = goal_y;
        goal_msg.pose.pose.orientation.w = 1.0;

        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending path planning request");
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const auto & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Path planning succeeded");
                this->publishPath(result.result->path);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Path planning failed");
            }
        };

        auto goal_handle_future = client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    void publishPath(const nav_msgs::msg::Path & path)
    {
        // In a full implementation, you would publish this path
        // to a visualization topic or use it for navigation
        RCLCPP_INFO(this->get_logger(), "Path with %zu points received", path.poses.size());
    }

    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr client_ptr_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BasicPathPlanner>();
    
    // Plan a path from (0,0) to (5,5)
    node->planPath(0.0, 0.0, 5.0, 5.0);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Example 2: Advanced Path Planning with Theta* Algorithm

This example demonstrates path planning with the Theta* algorithm, which allows for any-angle paths in grid environments.

### Configuration

```yaml
# theta_star_planning.yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_theta_star_planner::ThetaStarPlanner"
      tolerance: 0.5
      angle_tolerance: 0.5
      use_astar: true
      unknown_cost: 255
      lethal_cost: 254
      track_unknown_space: false
      max_iterations: 100000
      max_on_approach_iterations: 1000
```

### Implementation Code

```cpp
// theta_star_planner.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <memory>

class ThetaStarPlanner : public rclcpp::Node
{
public:
    ThetaStarPlanner() : Node("theta_star_planner")
    {
        // Initialize action client
        this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
            this, "compute_path_to_pose");
            
        RCLCPP_INFO(this->get_logger(), "Theta* Planner initialized");
    }

    void planPathToPose(double goal_x, double goal_y)
    {
        // Wait for the action server to be ready
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        // Create goal message
        auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = goal_x;
        goal_msg.pose.pose.position.y = goal_y;
        goal_msg.pose.pose.orientation.w = 1.0;

        // Send goal with custom planner selection
        RCLCPP_INFO(this->get_logger(), "Planning path with Theta* algorithm");
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const auto & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Theta* path planning succeeded");
                printPathMetrics(result.result->path);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Theta* path planning failed");
            }
        };

        auto goal_handle_future = client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    void printPathMetrics(const nav_msgs::msg::Path & path)
    {
        if (path.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path is empty");
            return;
        }

        // Calculate path length
        double length = 0.0;
        for (size_t i = 1; i < path.poses.size(); ++i) {
            const auto & p1 = path.poses[i-1].pose.position;
            const auto & p2 = path.poses[i].pose.position;
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            length += sqrt(dx*dx + dy*dy);
        }

        RCLCPP_INFO(this->get_logger(), 
                   "Path has %zu points, total length: %.2f meters", 
                   path.poses.size(), length);
        
        // Check for smoothness by calculating average angle between consecutive segments
        if (path.poses.size() > 2) {
            double total_angle = 0.0;
            int angle_count = 0;
            
            for (size_t i = 1; i < path.poses.size() - 1; ++i) {
                const auto & p0 = path.poses[i-1].pose.position;
                const auto & p1 = path.poses[i].pose.position;
                const auto & p2 = path.poses[i+1].pose.position;
                
                // Calculate vectors
                double v1_x = p1.x - p0.x;
                double v1_y = p1.y - p0.y;
                double v2_x = p2.x - p1.x;
                double v2_y = p2.y - p1.y;
                
                // Calculate angle between vectors
                double dot_product = v1_x * v2_x + v1_y * v2_y;
                double norms = sqrt(v1_x*v1_x + v1_y*v1_y) * sqrt(v2_x*v2_x + v2_y*v2_y);
                
                if (norms > 0.0) {
                    double angle = acos(std::max(-1.0, std::min(1.0, dot_product / norms)));
                    total_angle += angle;
                    angle_count++;
                }
            }
            
            if (angle_count > 0) {
                double avg_angle = total_angle / angle_count;
                RCLCPP_INFO(this->get_logger(), 
                           "Average turning angle: %.2f degrees", 
                           avg_angle * 180.0 / M_PI);
            }
        }
    }

    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr client_ptr_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThetaStarPlanner>();
    
    // Plan a path to a goal location
    node->planPathToPose(8.0, 8.0);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Example 3: Multi-Algorithm Path Planning with Comparison

This example demonstrates how to use different path planning algorithms and compare their outputs.

### Configuration

```yaml
# multi_algorithm_planning.yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["NavfnPlanner", "ThetaStarPlanner"]
    
    NavfnPlanner:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      
    ThetaStarPlanner:
      plugin: "nav2_theta_star_planner::ThetaStarPlanner"
      tolerance: 0.5
      angle_tolerance: 0.5
      use_astar: true
```

### Implementation Code

```cpp
// multi_algorithm_planner.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <vector>
#include <string>

class MultiAlgorithmPlanner : public rclcpp::Node
{
public:
    MultiAlgorithmPlanner() : Node("multi_algorithm_planner")
    {
        // Initialize action client
        this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
            this, "compute_path_to_pose");
            
        // Publisher for path comparison results
        this->comparison_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "path_comparison_results", 10);
            
        RCLCPP_INFO(this->get_logger(), "Multi-Algorithm Planner initialized");
    }

    void comparePlanningAlgorithms(double start_x, double start_y, double goal_x, double goal_y)
    {
        // Wait for the action server to be ready
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        // Store the goal for potential replanning
        start_pos_ = {start_x, start_y};
        goal_pos_ = {goal_x, goal_y};

        // Plan with different algorithms
        std::vector<std::string> algorithms = {"NavfnPlanner", "ThetaStarPlanner"};
        
        for (const auto & algorithm : algorithms) {
            this->requestPath(algorithm);
        }
    }

private:
    void requestPath(const std::string & algorithm)
    {
        // Create goal message
        auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = goal_pos_.first;
        goal_msg.pose.pose.position.y = goal_pos_.second;
        goal_msg.pose.pose.orientation.w = 1.0;

        // Add algorithm selection to goal (this is a conceptual example,
        // actual implementation would need Nav2 to support this)
        RCLCPP_INFO(this->get_logger(), "Requesting path with %s", algorithm.c_str());
        
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this, algorithm](const auto & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "%s path planning succeeded", algorithm.c_str());
                this->analyzePath(result.result->path, algorithm);
            } else {
                RCLCPP_ERROR(this->get_logger(), "%s path planning failed", algorithm.c_str());
            }
        };

        auto goal_handle_future = client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void analyzePath(const nav_msgs::msg::Path & path, const std::string & algorithm)
    {
        if (path.poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path for %s is empty", algorithm.c_str());
            return;
        }

        // Calculate path metrics
        double length = calculatePathLength(path);
        double smoothness = calculatePathSmoothness(path);
        int num_points = path.poses.size();
        
        // Create comparison report
        std::ostringstream report;
        report << "Algorithm: " << algorithm 
               << " | Points: " << num_points 
               << " | Length: " << std::fixed << std::setprecision(2) << length
               << "m | Smoothness: " << std::fixed << std::setprecision(2) << smoothness;
        
        RCLCPP_INFO(this->get_logger(), "%s", report.str().c_str());
        
        // Publish comparison result
        auto msg = std_msgs::msg::String();
        msg.data = report.str();
        comparison_publisher_->publish(msg);
    }

    double calculatePathLength(const nav_msgs::msg::Path & path)
    {
        double length = 0.0;
        for (size_t i = 1; i < path.poses.size(); ++i) {
            const auto & p1 = path.poses[i-1].pose.position;
            const auto & p2 = path.poses[i].pose.position;
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            length += sqrt(dx*dx + dy*dy);
        }
        return length;
    }
    
    double calculatePathSmoothness(const nav_msgs::msg::Path & path)
    {
        if (path.poses.size() < 3) return 0.0;
        
        double total_angle = 0.0;
        int angle_count = 0;
        
        for (size_t i = 1; i < path.poses.size() - 1; ++i) {
            const auto & p0 = path.poses[i-1].pose.position;
            const auto & p1 = path.poses[i].pose.position;
            const auto & p2 = path.poses[i+1].pose.position;
            
            // Calculate vectors
            double v1_x = p1.x - p0.x;
            double v1_y = p1.y - p0.y;
            double v2_x = p2.x - p1.x;
            double v2_y = p2.y - p1.y;
            
            // Calculate angle between vectors
            double dot_product = v1_x * v2_x + v1_y * v2_y;
            double norms = sqrt(v1_x*v1_x + v1_y*v1_y) * sqrt(v2_x*v2_x + v2_y*v2_y);
            
            if (norms > 0.0) {
                double angle = acos(std::max(-1.0, std::min(1.0, dot_product / norms)));
                total_angle += angle;
                angle_count++;
            }
        }
        
        return angle_count > 0 ? total_angle / angle_count : 0.0;
    }

    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr client_ptr_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr comparison_publisher_;
    std::pair<double, double> start_pos_, goal_pos_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiAlgorithmPlanner>();
    
    // Compare path planning algorithms for a given start-goal pair
    node->comparePlanningAlgorithms(0.0, 0.0, 10.0, 10.0);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Example 4: Isaac ROS-Integrated Path Planning with Perception

This example demonstrates how to integrate path planning with Isaac ROS perception data.

### Configuration

```yaml
# isaac_perception_planning.yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["IsaacEnhancedPlanner"]
    
    IsaacEnhancedPlanner:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: false  # Don't plan through unknown areas when using perception

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.025  # High resolution for detailed perception
      robot_radius: 0.22
      plugins: ["obstacle_layer", "inflation_layer", "isaac_perception_layer"]
      
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_range: 8.0
          obstacle_range: 7.0
          
      isaac_perception_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: isaac_detections
        isaac_detections:
          topic: /isaac_ros/detections
          max_obstacle_height: 2.0
          clearing: False  # Perception data typically doesn't clear
          marking: True
          data_type: "PointCloud2"
          raytrace_range: 10.0
          obstacle_range: 8.0
          
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0  # Higher for detailed obstacle avoidance
        inflation_radius: 0.7
```

### Implementation Code

```cpp
// isaac_perception_planner.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_util/robot_utils.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <memory>
#include <vector>

class IsaacPerceptionPlanner : public rclcpp::Node
{
public:
    IsaacPerceptionPlanner() : Node("isaac_perception_planner")
    {
        // Initialize action client for path planning
        this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
            this, "compute_path_to_pose");
            
        // Subscribers for Isaac ROS perception data
        this->detection_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/isaac_ros/detections", 10,
            std::bind(&IsaacPerceptionPlanner::detectionCallback, this, std::placeholders::_1));
            
        this->scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&IsaacPerceptionPlanner::scanCallback, this, std::placeholders::_1));
            
        // Publisher for enhanced path visualization
        this->enhanced_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(
            "/isaac_enhanced_path", 10);
            
        this->perception_data_available_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Isaac Perception Planner initialized");
    }

    void planPathWithPerception(double goal_x, double goal_y)
    {
        // Wait for perception data to be available
        if (!perception_data_available_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for perception data...");
            // In a real implementation, you might want to wait or use a timeout
        }
        
        // Wait for the action server to be ready
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        // Create goal message
        auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = goal_x;
        goal_msg.pose.pose.position.y = goal_y;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Planning path with Isaac perception enhancement");
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const auto & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Path planning with perception succeeded");
                // Process the path further if needed
                this->processEnhancedPath(result.result->path);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Path planning failed");
            }
        };

        auto goal_handle_future = client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    void detectionCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Process Isaac ROS detection data
        RCLCPP_DEBUG(this->get_logger(), "Received detection data with %d points", 
                    msg->width * msg->height);
        
        // Convert to PCL for processing
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
        
        // Filter the point cloud for better processing
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // 10cm voxels
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.filter(*filtered_cloud);
        
        RCLCPP_DEBUG(this->get_logger(), "Filtered detection data to %zu points", filtered_cloud->size());
        
        perception_data_available_ = true;
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process laser scan data
        RCLCPP_DEBUG(this->get_logger(), "Received laser scan with %zu ranges", msg->ranges.size());
        
        // Process scan data for integration with path planning
        // This is where you could implement custom processing that combines
        // laser and perception data for better path planning
    }
    
    void processEnhancedPath(const nav_msgs::msg::Path & original_path)
    {
        // Apply enhancement based on Isaac perception data
        nav_msgs::msg::Path enhanced_path = original_path;
        
        // Example enhancement: smooth the path based on perception confidence
        if (enhanced_path.poses.size() > 2) {
            // Implement path smoothing or other enhancements
            // based on perceived environment data
            enhanced_path = this->applyPathSmoothing(enhanced_path);
        }
        
        // Publish the enhanced path
        enhanced_path_publisher_->publish(enhanced_path);
        RCLCPP_INFO(this->get_logger(), "Published enhanced path with %zu points", 
                   enhanced_path.poses.size());
    }
    
    nav_msgs::msg::Path applyPathSmoothing(const nav_msgs::msg::Path & path)
    {
        // Implement a simple path smoothing algorithm
        nav_msgs::msg::Path smoothed_path;
        smoothed_path.header = path.header;
        
        if (path.poses.size() < 3) {
            return path; // Not enough points to smooth
        }
        
        // Add first point
        smoothed_path.poses.push_back(path.poses[0]);
        
        // Smooth intermediate points
        for (size_t i = 1; i < path.poses.size() - 1; ++i) {
            auto & prev = path.poses[i-1].pose.position;
            auto & curr = path.poses[i].pose.position;
            auto & next = path.poses[i+1].pose.position;
            
            // Simple averaging with neighbors
            geometry_msgs::msg::PoseStamped smoothed_pose;
            smoothed_pose.header = path.poses[i].header;
            smoothed_pose.pose.position.x = (prev.x + curr.x + next.x) / 3.0;
            smoothed_pose.pose.position.y = (prev.y + curr.y + next.y) / 3.0;
            smoothed_pose.pose.position.z = curr.z; // Keep z unchanged
            
            // Preserve orientation from original
            smoothed_pose.pose.orientation = path.poses[i].pose.orientation;
            
            smoothed_path.poses.push_back(smoothed_pose);
        }
        
        // Add last point
        if (!path.poses.empty()) {
            smoothed_path.poses.push_back(path.poses.back());
        }
        
        return smoothed_path;
    }

    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr client_ptr_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr detection_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr enhanced_path_publisher_;
    bool perception_data_available_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IsaacPerceptionPlanner>();
    
    // Plan a path with Isaac perception integration
    node->planPathWithPerception(6.0, 6.0);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Example 5: Dynamic Path Replanning with Isaac Simulation

This example demonstrates how to implement dynamic path replanning based on changing conditions detected by Isaac simulation.

### Implementation Code

```cpp
// dynamic_replanning.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <chrono>

class DynamicReplanner : public rclcpp::Node
{
public:
    DynamicReplanner() : Node("dynamic_replanner")
    {
        // Initialize action clients
        this->nav_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");
        this->plan_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
            this, "compute_path_to_pose");
        
        // Initialize transform listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Subscriber for sensor data to detect path blockages
        this->scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&DynamicReplanner::scanCallback, this, std::placeholders::_1));
        
        // Timer for periodic path checks
        this->path_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&DynamicReplanner::checkPathValidity, this));
        
        this->is_navigating_ = false;
        this->path_valid_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Dynamic Replanner initialized");
    }

    void navigateWithReplanning(double goal_x, double goal_y)
    {
        // Wait for action servers
        if (!nav_client_ptr_->wait_for_action_server(std::chrono::seconds(5)) ||
            !plan_client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action servers not available");
            return;
        }

        // Create navigation goal
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = goal_x;
        goal_msg.pose.pose.position.y = goal_y;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Starting navigation with dynamic replanning");
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const auto & result) {
            this->is_navigating_ = false;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Navigation failed");
            }
        };

        // Send the navigation goal
        auto goal_handle_future = nav_client_ptr_->async_send_goal(goal_msg, send_goal_options);
        is_navigating_ = true;
        
        // Store the goal for potential replanning
        goal_pose_ = goal_msg.pose;
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Check if the path ahead is blocked
        double min_range = 1.0; // Minimum safe distance in front of robot
        int front_scan_idx = msg->ranges.size() / 2; // Front of robot
        
        // Check several readings in front of robot
        int check_range = 10; // Number of readings to check
        int start_idx = std::max(0, front_scan_idx - check_range/2);
        int end_idx = std::min((int)msg->ranges.size(), front_scan_idx + check_range/2);
        
        bool obstacle_detected = false;
        for (int i = start_idx; i < end_idx; ++i) {
            if (msg->ranges[i] < min_range && msg->ranges[i] > msg->range_min) {
                obstacle_detected = true;
                break;
            }
        }
        
        if (obstacle_detected && is_navigating_ && path_valid_) {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected ahead, path may need replanning");
            path_valid_ = false;
        }
    }
    
    void checkPathValidity()
    {
        if (!is_navigating_ || path_valid_) {
            return; // Nothing to do
        }
        
        // Request a new path to the same goal
        RCLCPP_INFO(this->get_logger(), "Replanning path due to detected changes");
        
        auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
        goal_msg.pose = goal_pose_;
        
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const auto & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Replanning succeeded, path is valid again");
                path_valid_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Replanning failed");
                // In a real implementation, you might try different strategies
            }
        };

        auto goal_handle_future = plan_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_ptr_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr plan_client_ptr_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::TimerBase::SharedPtr path_check_timer_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    geometry_msgs::msg::PoseStamped goal_pose_;
    bool is_navigating_;
    bool path_valid_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicReplanner>();
    
    // Start navigation with dynamic replanning
    node->navigateWithReplanning(8.0, 8.0);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Isaac Sim Integration Examples

### Path Planning in Isaac Sim Environment

To test these path planning examples in Isaac Sim:

1. **Launch Isaac Sim with a navigation scenario**:
   ```bash
   cd /opt/nvidia/isaac_sim
   ./isaac-sim.sh --exec "omni.isaac.examples.simple_robots.carter_navigation"
   ```

2. **Run path planning node**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 run your_package_name basic_path_planning
   ```

3. **Visualize results in RViz2**:
   ```bash
   ros2 run rviz2 rviz2
   # Add displays for path visualization
   ```

## Performance Considerations

### For Isaac-Optimized Path Planning

1. **Leverage GPU acceleration** for path planning when possible
2. **Use appropriate costmap resolution** for your environment
3. **Optimize update frequencies** based on robot speed and environment dynamics
4. **Consider using multiple planning algorithms** with a selector based on environment types

### Memory and Computation Optimization

```cpp
// Efficient path planning implementation
class OptimizedPathPlanner : public rclcpp::Node
{
public:
    OptimizedPathPlanner() : Node("optimized_path_planner")
    {
        // Initialize with efficient data structures
        path_cache_.reserve(1000); // Reserve space for efficiency
        // Additional optimizations...
    }

private:
    std::vector<geometry_msgs::msg::Pose> path_cache_;
    // Implementation details...
};
```

These examples provide a comprehensive foundation for implementing path planning with Isaac tools, from basic algorithms to advanced perception-integrated and dynamic replanning systems. Each example builds upon the previous ones, demonstrating progressively more sophisticated approaches to path planning in the Isaac ecosystem.