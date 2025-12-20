# Chapter 8.4: Lab: Perception System Implementation

## Objective

Implement a complete perception system for a humanoid robot using Isaac Sim and Isaac ROS tools. Students will configure and integrate VSLAM, perception pipelines, and synthetic data generation to create a system that enables the robot to perceive and interpret its environment accurately.

## Materials Required

### Software
- Isaac Sim 2023.1.1
- Isaac ROS packages (Visual SLAM, Image Pipeline, Apriltag)
- ROS 2 Humble Hawksbill
- CUDA 12.x with appropriate GPU drivers
- Python 3.8-3.10

### Simulation Assets
- Humanoid robot model (e.g., Carter or similar)
- Camera sensors configured on the robot
- Sample environments for testing
- AprilTag markers for validation

## Setup Instructions

### Step 1: Environment Verification
1. Verify Isaac Sim is properly installed:
   ```bash
   cd /opt/nvidia/isaac_sim
   ./python.sh -c "import omni; print('Isaac Sim accessible')"
   ```

2. Verify Isaac ROS workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 pkg list | grep isaac
   ```

3. Check GPU access:
   ```bash
   python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
   ```

### Step 2: Launch Isaac Sim with Robot
1. Launch Isaac Sim with a humanoid robot (e.g., Carter):
   ```bash
   cd /opt/nvidia/isaac_sim
   ./isaac-sim.sh --exec "omni.isaac.examples.simple_robots.carter_franka_pick_place"
   ```

2. In Isaac Sim:
   - Verify the robot model loads correctly
   - Check that camera sensors are properly positioned and configured
   - Ensure the robot's URDF is loaded properly
   - Verify all joints are controllable

### Step 3: Configure ROS 2 Bridge
1. In Isaac Sim, enable the ROS Bridge extension:
   - Go to Window → Extensions
   - Search for "ROS Bridge" or "omni.isaac.ros_bridge"
   - Enable the extension

2. Verify ROS 2 connectivity:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 topic list | grep -E "(camera|joint|imu)"
   ```

## Procedures

### Procedure 1: Set Up VSLAM Pipeline
1. Create a launch file for the VSLAM system:
   ```bash
   mkdir -p ~/isaac_perception_lab/launch
   touch ~/isaac_perception_lab/launch/vslam_pipeline.launch.py
   ```

2. Add the following content to the launch file:
   ```python
   # ~/isaac_perception_lab/launch/vslam_pipeline.launch.py
   from launch import LaunchDescription
   from launch_ros.actions import Node
   from launch.actions import DeclareLaunchArgument
   from launch.substitutions import LaunchConfiguration

   def generate_launch_description():
       # Declare launch arguments
       use_sim_time = LaunchConfiguration('use_sim_time', default='True')
       
       # Isaac ROS Visual SLAM node
       visual_slam_node = Node(
           package='isaac_ros_visual_slam',
           executable='isaac_ros_visual_slam_node',
           parameters=[{
               'enable_occupancy_map': True,
               'occupancy_map_resolution': 0.05,
               'enable_localization': False,
               'use_sim_time': use_sim_time,
           }],
           remappings=[
               ('/camera/left/image_rect', '/camera/image_rect_color'),
               ('/camera/left/camera_info', '/camera/camera_info'),
               ('/camera/right/image_rect', '/camera/image_rect_color'),  # For stereo
               ('/camera/right/camera_info', '/camera/camera_info'),
           ]
       )
       
       # Image rectification node
       rectify_node = Node(
           package='isaac_ros_image_proc',
           executable='isaac_ros_rectify_node',
           name='rectify_node',
           parameters=[{
               'input_width': 640,
               'input_height': 480,
               'output_width': 640,
               'output_height': 480,
               'use_sim_time': use_sim_time,
           }],
           remappings=[
               ('image_raw', '/camera/image_raw'),
               ('camera_info', '/camera/camera_info'),
               ('image_rect', '/camera/image_rect_color'),
           ]
       )

       return LaunchDescription([
           rectify_node,
           visual_slam_node
       ])
   ```

3. Launch the VSLAM pipeline:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 launch ~/isaac_perception_lab/launch/vslam_pipeline.launch.py
   ```

4. Expected outcome: Both the rectification and VSLAM nodes start without errors and begin processing camera data.

### Procedure 2: Implement Multi-Component Perception Pipeline
1. Create a comprehensive perception pipeline launch file:
   ```bash
   touch ~/isaac_perception_lab/launch/perception_pipeline.launch.py
   ```

2. Add content for a multi-component pipeline:
   ```python
   # ~/isaac_perception_lab/launch/perception_pipeline.launch.py
   from launch import LaunchDescription
   from launch_ros.actions import ComposableNodeContainer
   from launch_ros.descriptions import ComposableNode

   def generate_launch_description():
       # Create a container for the perception pipeline
       perception_container = ComposableNodeContainer(
           name='perception_container',
           namespace='',
           package='rclcpp_components',
           executable='component_container_mt',
           composable_node_descriptions=[
               # Image rectification
               ComposableNode(
                   package='isaac_ros_image_proc',
                   plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                   name='image_rectify_node',
                   parameters=[{
                       'input_width': 640,
                       'input_height': 480,
                       'output_width': 640,
                       'output_height': 480,
                   }],
                   remappings=[
                       ('image_raw', '/camera/image_raw'),
                       ('camera_info', '/camera/camera_info'),
                       ('image_rect', '/camera/image_rect_color'),
                   ],
               ),
               
               # AprilTag detection
               ComposableNode(
                   package='isaac_ros_apriltag',
                   plugin='nvidia::isaac_ros::apriltag::ApriltagNode',
                   name='apriltag_node',
                   parameters=[{
                       'family': '36h11',
                       'max_tags': 10,
                       'publish_tag_detections_image': True,
                   }],
                   remappings=[
                       ('image', '/camera/image_rect_color'),
                       ('camera_info', '/camera/camera_info'),
                   ],
               ),
               
               # Visual SLAM
               ComposableNode(
                   package='isaac_ros_visual_slam',
                   plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                   name='visual_slam_node',
                   parameters=[{
                       'enable_occupancy_map': True,
                       'occupancy_map_resolution': 0.05,
                       'enable_localization': False,
                   }],
                   remappings=[
                       ('camera/left/image_rect', '/camera/image_rect_color'),
                       ('camera/left/camera_info', '/camera/camera_info'),
                   ],
               ),
           ],
           output='screen',
       )

       return LaunchDescription([perception_container])
   ```

3. Launch the comprehensive pipeline:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 launch ~/isaac_perception_lab/launch/perception_pipeline.launch.py
   ```

4. Expected outcome: All perception components start, process data, and publish results to appropriate topics.

### Procedure 3: Integrate Perception with Robot Control
1. Create a robot controller that uses perception data:
   ```bash
   mkdir -p ~/isaac_perception_lab/src
   touch ~/isaac_perception_lab/src/perception_controller.py
   ```

2. Add the following content:
   ```python
   #!/usr/bin/env python3
   # ~/isaac_perception_lab/src/perception_controller.py
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import Image
   from geometry_msgs.msg import Twist, PoseStamped
   from visualization_msgs.msg import MarkerArray
   import cv2
   from cv2 import aruco
   from std_msgs.msg import String
   import numpy as np
   from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

   class PerceptionController(Node):
       def __init__(self):
           super().__init__('perception_controller')
           
           # Create a QoS profile for sensor data
           sensor_qos = QoSProfile(
               depth=10,
               durability=QoSDurabilityPolicy.VOLATILE,
               reliability=QoSReliabilityPolicy.BEST_EFFORT
           )
           
           # Create subscribers
           self.image_subscription = self.create_subscription(
               Image,
               '/camera/image_rect_color',
               self.image_callback,
               sensor_qos)
               
           self.pose_subscription = self.create_subscription(
               PoseStamped,
               '/visual_slam/pose',
               self.pose_callback,
               10)
               
           self.tag_subscription = self.create_subscription(
               MarkerArray,
               '/apriltag_detections',
               self.tag_callback,
               10)
           
           # Create publishers
           self.cmd_publisher = self.create_publisher(
               Twist,
               '/cmd_vel',
               10)
               
           self.status_publisher = self.create_publisher(
               String,
               '/perception_status',
               10)
           
           # Initialize CV bridge (if needed for image processing)
           self.get_logger().info('Perception Controller initialized')
           
       def image_callback(self, msg):
           # Process image data
           self.get_logger().info(f'Received image: {msg.width}x{msg.height}')
           
       def pose_callback(self, msg):
           # Process pose data
           self.get_logger().info(f'Received pose at: {msg.pose.position.x}, {msg.pose.position.y}')
           
       def tag_callback(self, msg):
           # Process AprilTag detections
           if len(msg.markers) > 0:
               self.get_logger().info(f'Detected {len(msg.markers)} tags')
               # Publish status about tag detection
               status_msg = String()
               status_msg.data = f"Detected {len(msg.markers)} tags"
               self.status_publisher.publish(status_msg)
               
       def send_command(self, linear_x, angular_z):
           cmd = Twist()
           cmd.linear.x = linear_x
           cmd.angular.z = angular_z
           self.cmd_publisher.publish(cmd)

   def main(args=None):
       rclpy.init(args=args)
       perception_controller = PerceptionController()
       
       try:
           rclpy.spin(perception_controller)
       except KeyboardInterrupt:
           pass
       finally:
           perception_controller.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. Make the controller executable and run it:
   ```bash
   chmod +x ~/isaac_perception_lab/src/perception_controller.py
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   python3 ~/isaac_perception_lab/src/perception_controller.py
   ```

### Procedure 4: Create Synthetic Data Generation Script
1. Create a synthetic data generation script:
   ```bash
   touch ~/isaac_perception_lab/scripts/synthetic_data_generator.py
   ```

2. Add the following content:
   ```python
   #!/usr/bin/env python3
   # ~/isaac_perception_lab/scripts/synthetic_data_generator.py
   import omni
from omni.isaac.kit import SimulationApp
   import numpy as np
   import random
   import os
   import json
   from PIL import Image
   import carb
   import omni.replicator.core as rep

   # Initialize Isaac Sim in headless mode
   simulation_app = SimulationApp({"headless": False})  # Set to True for headless

   from omni.isaac.core import World
   from omni.isaac.core.utils import stage, light as light_utils, geometry as geo_utils
   from omni.isaac.synthetic_utils import SyntheticDataHelper
   from omni.isaac.core.utils.stage import add_reference_to_stage
   from omni.isaac.core.prims import VisualMeshPrim
   import omni.isaac.core.utils.prims as prim_utils


   class SyntheticDataGenerator:
       def __init__(self, output_dir="./synthetic_data"):
           self.output_dir = output_dir
           self.world = World(stage_units_in_meters=1.0)
           self.setup_scene()
           self.setup_cameras()
           self.setup_synthetic_data()
           os.makedirs(output_dir, exist_ok=True)

       def setup_scene(self):
           """Setup the basic scene with lights and ground plane"""
           # Add ground plane
           geo_utils.create_ground_plane("/World/GroundPlane", "x", 10, 0, 0.5, False)
           
           # Add dome light
           light_utils.create_dome_light("/World/DomeLight", color=[0.1, 0.1, 0.1], intensity=3000)
           
           # Add directional light
           light_utils.create_directional_light("/World/DirectionalLight", 
                                               color=[1.0, 1.0, 1.0], 
                                               intensity=1500, 
                                               position=[10, 10, 5],
                                               direction=[-1, -1, -1])

       def setup_cameras(self):
           """Setup the camera for synthetic data capture"""
           # Create a camera prim
           self.camera_path = "/World/Camera"
           self.camera_prim = prim_utils.create_prim(
               self.camera_path,
               "Camera",
               position=[0.0, 0.0, 1.0],
               orientation=[0.0, 0.0, 0.0, 1.0]
           )
           
           # Set camera properties
           from pxr import Gf
           camera_prim = self.camera_prim
           camera_prim.GetAttribute("focallength").Set(24)
           camera_prim.GetAttribute("clippingRange").Set(Gf.Vec2f(0.1, 10000))
           camera_prim.GetAttribute("horizontalAperture").Set(20.955)
           camera_prim.GetAttribute("verticalAperture").Set(15.2908)

       def setup_synthetic_data(self):
           """Setup synthetic data capture"""
           self.viewport_name = "Viewport"
           self.sd_helper = SyntheticDataHelper(
               viewport_name=self.viewport_name,
               resolution=(640, 480)
           )

       def randomize_scene(self):
           """Randomize elements in the scene"""
           # Randomize lighting
           dome_light = self.world.scene.get_object("/World/DomeLight")
           intensity = random.uniform(1000, 5000)
           dome_light.set_attribute("inputs:intensity", intensity)
           
           # Add random objects
           num_objects = random.randint(3, 8)
           for i in range(num_objects):
               # Random position
               x = random.uniform(-4, 4)
               y = random.uniform(-4, 4)
               z = random.uniform(0.1, 2)
               
               # Random object type
               obj_types = ["Cube", "Cylinder", "Sphere"]
               obj_type = random.choice(obj_types)
               
               # Random color
               color = [random.random(), random.random(), random.random()]
               
               # Add object with specific name
               obj_name = f"/World/RandomObject_{i}"
               if obj_type == "Cube":
                   geo_utils.create_box(obj_name, position=[x, y, z], size=0.5, color=color)
               elif obj_type == "Cylinder":
                   geo_utils.create_cylinder(obj_name, position=[x, y, z], radius=0.25, height=0.5, color=color)
               elif obj_type == "Sphere":
                   geo_utils.create_sphere(obj_name, position=[x, y, z], radius=0.25, color=color)

       def capture_frame(self, frame_idx):
           """Capture a single frame of synthetic data"""
           # Step the world to ensure rendering
           self.world.step(render=True)
           
           # Get data from synthetic helper
           try:
               # Get RGB image
               rgb_data = self.sd_helper.get_rgb()
               
               # Get depth information
               depth_data = self.sd_helper.get_depth()
               
               # Get segmentation
               seg_data = self.sd_helper.get_semantic_segmentation()
               
               # Save data
               self.save_frame_data(rgb_data, depth_data, seg_data, frame_idx)
               
               print(f"Saved frame {frame_idx}")
           except Exception as e:
               print(f"Error capturing frame {frame_idx}: {e}")

       def save_frame_data(self, rgb_data, depth_data, seg_data, frame_idx):
           """Save captured frame data"""
           frame_dir = os.path.join(self.output_dir, f"frame_{frame_idx:04d}")
           os.makedirs(frame_dir, exist_ok=True)
           
           # Save RGB image
           if rgb_data is not None:
               rgb_img = Image.fromarray(rgb_data)
               rgb_img.save(os.path.join(frame_dir, "rgb.png"))
           
           # Save depth data
           if depth_data is not None:
               depth_path = os.path.join(frame_dir, "depth.npy")
               np.save(depth_path, depth_data)
           
           # Save segmentation
           if seg_data is not None:
               seg_img = Image.fromarray(seg_data.astype(np.uint8))
               seg_img.save(os.path.join(frame_dir, "segmentation.png"))
           
           # Save metadata
           metadata = {
               "frame_idx": frame_idx,
               "resolution": [640, 480],
               "camera_params": {
                   "focal_length": 24,
                   "horizontal_aperture": 20.955,
                   "vertical_aperture": 15.2908
               }
           }
           metadata_path = os.path.join(frame_dir, "metadata.json")
           with open(metadata_path, 'w') as f:
               json.dump(metadata, f)

       def generate_dataset(self, num_frames=100):
           """Generate a complete synthetic dataset"""
           for i in range(num_frames):
               # Clean up scene from previous iteration
               for prim_path in [p for p in self.world.scene.objects if "RandomObject" in p.name]:
                   self.world.scene.remove_object(prim_path.name)
               
               # Randomize scene
               self.randomize_scene()
               
               # Capture frame
               self.capture_frame(i)
               
               # Print progress
               if (i + 1) % 10 == 0:
                   print(f"Progress: {i + 1}/{num_frames} frames completed")

       def close(self):
           """Close the simulation"""
           self.world.clear()
           simulation_app.close()


   def main():
       generator = SyntheticDataGenerator(output_dir="./synthetic_data")
       generator.generate_dataset(num_frames=50)  # Generate 50 frames
       generator.close()


   if __name__ == "__main__":
       main()
   ```

3. Run the synthetic data generation:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   python3 ~/isaac_perception_lab/scripts/synthetic_data_generator.py
   ```

## Assessment Criteria

### Completion Requirements
- [ ] Successfully launch Isaac Sim with humanoid robot
- [ ] Configure and launch VSLAM pipeline
- [ ] Implement multi-component perception pipeline
- [ ] Integrate perception with robot control
- [ ] Generate synthetic data using Isaac Sim
- [ ] Demonstrate perception system in operation
- [ ] Validate perception outputs using appropriate metrics

### Performance Metrics
- [ ] VSLAM pipeline processes data at 10Hz or higher
- [ ] Perception system demonstrates accurate environmental mapping
- [ ] Synthetic data generation creates 50+ diverse frames
- [ ] System maintains stability during extended operation
- [ ] All components properly integrated without errors

### Verification Steps
1. Run VSLAM verification:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/isaac_ros_ws/install/setup.bash
   ros2 topic echo /visual_slam/pose --field pose.position.x | head -n 10
   ```

2. Check perception pipeline performance:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic hz /camera/image_rect_color
   ```

3. Validate synthetic data generation:
   ```bash
   ls -la ./synthetic_data/ | grep -c "frame_"
   ```

## Lab Report Requirements

Students must submit a lab report documenting:

1. **Setup Process**: Detailed account of the perception system setup, including any challenges encountered and solutions applied.

2. **Pipeline Implementation**: Description of the multi-component perception pipeline and configuration of each component.

3. **Integration Results**: Results of integrating perception with robot control, including any modifications needed for successful integration.

4. **Synthetic Data Generation**: Details of the synthetic data generation process, including diversity of scenes created.

5. **Performance Analysis**: Analysis of the system's performance metrics, including processing rates, accuracy, and resource utilization.

6. **Troubleshooting**: Any issues encountered during the lab and how they were resolved.

7. **Validation**: Evidence that the perception system meets the requirements for accurate environmental recognition and mapping.

## Extensions

### Advanced Perception Tasks
For advanced students:
1. Implement a more complex perception pipeline with additional sensors (LiDAR, IMU)
2. Integrate deep learning models with Isaac ROS perception pipeline
3. Create a perception system for a specific task (object manipulation, navigation assistance)

### Performance Optimization
1. Optimize the perception pipeline using NITROS for reduced latency
2. Implement dynamic adjustment of processing parameters based on scene complexity
3. Add performance monitoring and logging to the perception system

### Sim-to-Real Considerations
1. Apply domain randomization techniques to make the perception system more robust
2. Implement validation strategies to assess sim-to-real transfer capability
3. Create a hybrid system that can operate in both simulation and real-world environments

## Troubleshooting Guide

### Common Issues and Solutions

1. **VSLAM node fails to start**
   - Check GPU access: `python3 -c "import torch; print(torch.cuda.is_available())"`
   - Verify Isaac ROS packages are built: `ls ~/isaac_ros_ws/install/`
   - Ensure camera topics are available: `ros2 topic list | grep camera`

2. **Low processing frequency**
   - Monitor GPU utilization: `nvidia-smi`
   - Reduce image resolution if needed
   - Check for CPU bottlenecks with `htop`

3. **Perception outputs are unstable**
   - Verify camera calibration parameters
   - Check robot movement speed (too fast can cause tracking failure)
   - Adjust VSLAM parameters for your environment

4. **Synthetic data generation errors**
   - Ensure Isaac Sim is properly initialized
   - Check available disk space
   - Verify all required Isaac Sim extensions are enabled

5. **Integration issues between components**
   - Check topic names and remappings
   - Ensure consistent coordinate frames
   - Verify QoS settings match between publishers and subscribers