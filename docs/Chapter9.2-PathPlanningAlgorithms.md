# Chapter 9.2: Path Planning Algorithms

## Learning Objectives
- Understand fundamental path planning algorithms used in robotics
- Explain how different algorithms apply to Nav2 and Isaac-based systems
- Implement path planning algorithms for various environments
- Compare algorithm performance under different conditions
- Optimize path planning for real-time navigation in Isaac environments

## Estimated Completion Time: 3 hours

## Prerequisites
- Understanding of Nav2 architecture (covered in Chapter 9.1)
- Basic knowledge of graph theory and algorithms
- Familiarity with ROS 2 navigation concepts
- Understanding of coordinate systems and transforms

## Introduction to Path Planning

Path planning is a fundamental component of robotic navigation that determines optimal or feasible paths from a starting point to a goal while avoiding obstacles. In the context of Isaac-based robotic systems, path planning algorithms must efficiently process high-fidelity environment information and integrate with GPU-accelerated perception systems.

Path planning algorithms can be classified along several dimensions:
1. **Global vs. Local**: Global planners create complete paths from start to goal; local planners focus on immediate obstacles
2. **Grid-based vs. Sampling-based**: Grid-based operate on discretized space; sampling-based work in continuous space
3. **Optimal vs. Feasible**: Optimal algorithms guarantee the best solution; feasible algorithms find any valid solution

## Grid-Based Path Planning Algorithms

### A* Algorithm

A* is one of the most popular grid-based path planning algorithms that combines the advantages of Dijkstra's algorithm with a heuristic function to guide search toward the goal.

**Algorithm Overview:**
1. Initialize open and closed lists
2. Add start node to open list with f-cost
3. While open list is not empty:
   - Select node with lowest f-cost (f = g + h)
   - If goal is reached, reconstruct path
   - Move current node to closed list
   - Examine neighbors and update costs

**Implementation in Nav2:**
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      # NavfnPlanner is based on Dijkstra's algorithm but can be adapted for A*
```

**Isaac-Specific Considerations:**
- A* works well with costmaps generated from Isaac's high-resolution sensors
- The heuristic function should account for Isaac Sim's accurate distance measurements
- Optimized implementations can leverage GPU acceleration for faster planning

### Dijkstra's Algorithm

Dijkstra's algorithm finds the shortest path from a source to all other nodes in a weighted graph. While less efficient than A* in most cases, it guarantees the optimal path.

**Key Differences from A*:**
- No heuristic function (h = 0)
- Explores in all directions equally
- Guarantees optimal path but takes longer

**Nav2 Configuration:**
```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"  # Uses a Dijkstra-like approach
  tolerance: 0.5  # Allow planning in proximity to obstacles
  use_astar: false  # Explicitly use Dijkstra's algorithm
```

### Jump Point Search (JPS)

Jump Point Search is an optimization of A* that exploits uniform-cost grid spaces to reduce the number of nodes expanded during pathfinding.

**Key Benefits:**
- Dramatically faster than A* in uniform-cost environments
- Maintains optimality
- Particularly effective for large open spaces

**Implementation Considerations:**
- Works best with grid-based maps
- Less beneficial in highly cluttered environments
- Can be combined with Nav2's grid-based planners

### Theta* Algorithm

Theta* is an any-angle path planning algorithm that allows for more natural paths than grid-constrained algorithms.

**Advantages:**
- Produces more natural, direct paths
- Maintains optimality in unconstrained space
- Better for robot dynamics

**Considerations for Isaac:**
- Works well when Isaac's environment provides accurate distance computations
- Requires more complex collision checking than grid-based approaches

## Sampling-Based Path Planning Algorithms

### Probabilistic Roadmap (PRM)

PRM pre-computes a roadmap of the environment and uses it for path planning. It works well for complex, static environments.

**Process:**
1. Sample points randomly in the configuration space
2. Connect nearby points that are collision-free
3. Use graph search to find path through roadmap

**Isaac Integration:**
- Can work with Isaac Sim's accurate geometric models
- Pre-computation can be done in simulation with real-world environments represented

### Rapidly-exploring Random Trees (RRT)

RRT is designed for high-dimensional spaces and complex robot kinematics.

**Process:**
1. Start with initial configuration
2. Randomly sample configuration space
3. Grow tree toward samples
4. Connect tree to goal when possible

**Variants:**
- **RRT***: Asymptotically optimal version of RRT
- **Bi-directional RRT**: Grows trees from both start and goal
- **RRT-Connect**: Bi-directional with direct connection attempts

### Informed RRT*

An improvement over RRT* that restricts sampling to an ellipsoidal region containing potential solutions.

**Advantages:**
- Faster convergence to optimal solution
- More efficient than basic RRT*
- Particularly useful for complex environments

## Nav2's Path Planning Architecture

### Plugin-Based System

Nav2 uses a plugin-based architecture for path planning, allowing different algorithms to be used interchangeably:

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased", "GridBasedInformed"]
    
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      
    GridBasedInformed:
      plugin: "nav2_theta_star_planner::ThetaStarPlanner"
      tolerance: 0.5
      angle_tolerance: 0.5
      use_astar: true
      unknown_cost: 255
      lethal_cost: 254
      track_unknown_space: false
```

### Available Planners in Nav2

1. **NavfnPlanner**: Grid-based planner using Dijkstra's algorithm
2. **ThetaStarPlanner**: Grid-based any-angle planner
3. **GlobalPlanner**: Alternative grid-based implementation
4. **LazyThetaStarPlanner**: Lazy evaluation variant of Theta*

## Algorithm Selection for Different Scenarios

### Indoor Navigation

For structured indoor environments, the following approaches work well:
- **A* or Navfn**: For static obstacle avoidance
- **Theta* or Lazy Theta***: For smoother, more natural paths
- **Pre-computed maps**: For frequently traveled routes

**Configuration Example:**
```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_theta_star_planner::ThetaStarPlanner"
      use_astar: true
      # Allow for more natural paths
      angle_tolerance: 0.2
      costmap_converter_plugin: "costmap_converter::CostmapToDynamicObstacles"
```

### Dynamic Environments

For environments with moving obstacles:
- **Local planners**: Handle dynamic obstacle avoidance
- **Replanning**: Frequent path updates
- **Predictive planning**: Consider motion of dynamic obstacles

### Complex Outdoor Environments

For outdoor navigation with varied terrain:
- **Multi-level planning**: Hierarchical approach
- **Terrain awareness**: Consider elevation and traversability
- **Visual planning**: Use Isaac ROS perception for path adjustment

## Isaac-Specific Path Planning Considerations

### High-Fidelity Environment Data

Isaac Sim provides precise geometric and texture information that can be leveraged for more accurate path planning:

1. **Accurate Geometry**: Use Isaac's precise collision meshes
2. **Detailed Costmaps**: Generate high-resolution costmaps from Isaac perception
3. **Semantic Information**: Use Isaac's semantic segmentation for terrain classification

### GPU Acceleration

Isaac's GPU acceleration can be applied to path planning through:

1. **Parallel Processing**: Accelerate grid-based planning algorithms
2. **Collision Checking**: Fast collision detection using GPU
3. **Costmap Creation**: Rapid generation of costmaps from sensor data

### Sim-to-Real Transfer

Path planning algorithms must account for differences between simulation and reality:

1. **Conservative Planning**: Account for simulation inaccuracies
2. **Robust Algorithms**: Use algorithms that work well under uncertainty
3. **Adaptive Planning**: Adjust planning parameters based on environment confidence

## Advanced Path Planning Techniques

### Multi-Modal Path Planning

Combine different planning approaches for complex tasks:
- **Hierarchical Planning**: Coarse global plan with fine local adjustments
- **Multi-Objective Planning**: Consider multiple criteria (distance, energy, safety)
- **Time-Dependent Planning**: Account for temporal changes in environment

### Learning-Based Path Planning

Integrate machine learning with traditional algorithms:
- **Reinforcement Learning**: Learn planning strategies through experience
- **Imitation Learning**: Learn from expert demonstrations
- **Deep Learning**: Use neural networks for environment understanding

## Implementation Guide

### Implementing a Custom Path Planner Plugin

To implement a custom path planner for Nav2:

```cpp
// custom_path_planner.hpp
#ifndef CUSTOM_PATH_PLANNER_HPP_
#define CUSTOM_PATH_PLANNER_HPP_

#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/robot_utils.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace custom_planners
{

class CustomPathPlanner : public nav2_core::GlobalPlanner
{
public:
    CustomPathPlanner() = default;
    ~CustomPathPlanner() override = default;
    
    // Initialize the planner
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        const std::shared_ptr<tf2_ros::Buffer> & tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;
    
    // Cleanup planner
    void cleanup() override;
    
    // Activate planner
    void activate() override;
    
    // Deactivate planner
    void deactivate() override;
    
    // Create path from start to goal
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;
    
private:
    // Planner-specific parameters
    std::string name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

} // namespace custom_planners

#endif // CUSTOM_PATH_PLANNER_HPP_
```

### Performance Optimization

```cpp
// Optimized A* implementation using priority queue
#include <queue>
#include <vector>

struct Node {
    double x, y;
    double g, h, f;
    int parent_index;
    
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

nav_msgs::msg::Path CustomPathPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = node_->now();
    
    // Get costmap dimensions and resolution
    auto costmap = costmap_ros_->getCostmap();
    double resolution = costmap->getResolution();
    
    // Convert poses to costmap coordinates
    int start_x = (start.pose.position.x - costmap->getOriginX()) / resolution;
    int start_y = (start.pose.position.y - costmap->getOriginY()) / resolution;
    int goal_x = (goal.pose.position.x - costmap->getOriginX()) / resolution;
    int goal_y = (goal.pose.position.y - costmap->getOriginY()) / resolution;
    
    // Priority queue for A* algorithm
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    
    // Visited node tracking
    std::vector<std::vector<bool>> closed_list(
        costmap->getSizeInCellsY(), 
        std::vector<bool>(costmap->getSizeInCellsX(), false)
    );
    
    // Initialize with start node
    Node start_node;
    start_node.x = start_x;
    start_node.y = start_y;
    start_node.g = 0.0;
    start_node.h = heuristic(start_x, start_y, goal_x, goal_y);
    start_node.f = start_node.g + start_node.h;
    start_node.parent_index = -1;
    
    open_list.push(start_node);
    
    // A* algorithm loop
    while (!open_list.empty()) {
        Node current = open_list.top();
        open_list.pop();
        
        // Mark as visited
        closed_list[(int)current.y][(int)current.x] = true;
        
        // Check if goal is reached
        if (current.x == goal_x && current.y == goal_y) {
            return reconstructPath(current, costmap, resolution);
        }
        
        // Explore neighbors
        exploreNeighbors(current, goal_x, goal_y, costmap, open_list, closed_list);
    }
    
    // No path found
    return path;
}
```

## Path Smoothing and Optimization

### Spline-Based Smoothing

To make paths smoother for robot execution:

```cpp
nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& original_path, double smoothing_strength = 1.0) {
    nav_msgs::msg::Path smoothed_path;
    smoothed_path.header = original_path.header;
    
    if (original_path.poses.size() < 3) {
        return original_path;  // Not enough points to smooth
    }
    
    // Apply smoothing algorithm (e.g., cubic splines)
    std::vector<double> x_coords, y_coords;
    for (const auto& pose : original_path.poses) {
        x_coords.push_back(pose.pose.position.x);
        y_coords.push_back(pose.pose.position.y);
    }
    
    // Apply spline interpolation to create smoother path
    // Implementation would use a spline library like splipy or create custom implementation
    
    return smoothed_path;
}
```

### Dynamic Window Approach Integration

For local path planning, integrate with local planners that use dynamic window approach:

```yaml
local_planner:
  ros__parameters:
    # Use in conjunction with global planner
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "nav2_mppi_controller::MPPITrajectoryController"
      # Model Predictive Path Integral controller for dynamic environments
```

## Performance Evaluation

### Algorithm Performance Metrics

1. **Computation Time**: Time to generate a valid path
2. **Path Quality**: Length, smoothness, and safety of the path
3. **Success Rate**: Percentage of successful path planning attempts
4. **Memory Usage**: Memory consumed during planning
5. **Robustness**: Performance across diverse environments

### Benchmarking with Isaac Sim

Isaac Sim provides controlled environments for algorithm benchmarking:

1. **Consistent Environments**: Reproducible test scenarios
2. **Ground Truth**: Accurate measurement of path quality
3. **Performance Monitoring**: Real-time computation tracking
4. **Variability Testing**: Test under different conditions

### Isaac-Specific Evaluation Code

```python
# performance_evaluation.py
import time
import numpy as np
from scipy.spatial.distance import euclidean

def benchmark_path_planner(planner, environment, start_pos, goal_pos):
    """Benchmark a path planning algorithm"""
    start_time = time.time()
    
    # Execute path planning
    path = planner.createPlan(start_pos, goal_pos)
    
    end_time = time.time()
    computation_time = end_time - start_time
    
    # Evaluate path quality
    path_length = calculate_path_length(path)
    path_smoothness = calculate_path_smoothness(path)
    path_validity = is_path_collision_free(path, environment)
    
    results = {
        'computation_time': computation_time,
        'path_length': path_length,
        'path_smoothness': path_smoothness,
        'path_validity': path_validity,
        'success': bool(path.poses) and path_validity
    }
    
    return results

def calculate_path_length(path):
    """Calculate the total length of a path"""
    if len(path.poses) < 2:
        return 0.0
    
    length = 0.0
    for i in range(1, len(path.poses)):
        p1 = [path.poses[i-1].pose.position.x, path.poses[i-1].pose.position.y]
        p2 = [path.poses[i].pose.position.x, path.poses[i].pose.position.y]
        length += euclidean(p1, p2)
    
    return length

def calculate_path_smoothness(path):
    """Calculate a measure of path smoothness"""
    if len(path.poses) < 3:
        return 0.0
    
    total_curvature = 0.0
    for i in range(1, len(path.poses) - 1):
        p1 = np.array([path.poses[i-1].pose.position.x, path.poses[i-1].pose.position.y])
        p2 = np.array([path.poses[i].pose.position.x, path.poses[i].pose.position.y])
        p3 = np.array([path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y])
        
        # Calculate curvature using three consecutive points
        v1 = p2 - p1
        v2 = p3 - p2
        angle = np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0))
        total_curvature += abs(angle)
    
    return total_curvature / (len(path.poses) - 2)
```

## Troubleshooting Path Planning Issues

### Common Issues and Solutions

1. **No Path Found**:
   - **Check**: Costmap and obstacle representation
   - **Solution**: Adjust inflation radius or planner tolerance

2. **Suboptimal Paths**:
   - **Check**: Algorithm parameters and heuristic function
   - **Solution**: Tune parameters or try different algorithms

3. **Performance Issues**:
   - **Check**: Computation time and memory usage
   - **Solution**: Optimize implementation or use different algorithms

4. **Collision Issues**:
   - **Check**: Robot footprint and costmap resolution
   - **Solution**: Adjust robot dimensions or costmap parameters

### Performance Debugging Commands

```bash
# Monitor planner performance
ros2 topic echo /planner_server/plan

# Check costmap status
ros2 service call /global_costmap/get_costmap nav2_msgs/srv/GetCostmap

# View planning status
ros2 lifecycle list planner_server
```

## Troubleshooting Tips

### Algorithm Selection
- Use A*/Navfn for general-purpose planning
- Use Theta* for smoother paths in grid environments
- Consider sampling-based methods for high-dimensional spaces
- Adjust parameters based on environment complexity

### Performance Optimization
- Preprocess maps when possible
- Use appropriate resolution for your application
- Implement efficient collision checking
- Consider hierarchical planning for complex environments

### Isaac-Specific Considerations
- Leverage Isaac's high-fidelity environment representation
- Use GPU acceleration when available
- Account for sim-to-real transfer challenges
- Validate performance in both simulation and reality

## Knowledge Check

1. What is the difference between global and local path planning?
2. Explain the advantages of A* over Dijkstra's algorithm.
3. What is Jump Point Search, and when is it most effective?
4. How does Nav2's plugin-based architecture benefit path planning?
5. What are the key considerations for path planning in Isaac-based systems?

Answers:
1. Global planning creates complete paths from start to goal, while local planning focuses on immediate obstacles and trajectory following.
2. A* uses a heuristic function to guide the search toward the goal, making it more efficient than Dijkstra's algorithm.
3. Jump Point Search is an optimization of A* that reduces nodes expanded in uniform-cost grids, making it most effective in large open spaces.
4. The plugin-based architecture allows different algorithms to be used interchangeably without changing the overall system architecture.
5. Key considerations include leveraging Isaac's high-fidelity data, using GPU acceleration, and accounting for sim-to-real differences.

## Lab Preparation

Before proceeding to the practical lab exercises in the next section, ensure you have:
- Understanding of different path planning algorithms and their characteristics
- Nav2 installed with path planning plugins
- Isaac Sim environment with navigation scenarios
- Knowledge of ROS 2 navigation message types
- Basic understanding of costmap generation and configuration