# Synthetic Data Generation Examples

## Overview

This document provides practical examples of synthetic data generation using Isaac Sim. These examples demonstrate different approaches to creating training datasets for AI-robotics applications, from simple object detection scenarios to complex multi-modal environments.

## Example 1: Basic Object Detection Dataset

This example generates a dataset for training an object detection model, with RGB images and corresponding bounding box annotations.

### Setup Script
```python
# basic_object_detection.py
import omni
from omni.isaac.kit import SimulationApp
import numpy as np
import random
import os
import json
from PIL import Image
import omni.replicator.core as rep
from pxr import Gf

# Initialize Isaac Sim
simulation_app = SimulationApp({"headless": False})  # Set to True for headless

from omni.isaac.core import World
from omni.isaac.core.utils import stage, light as light_utils, geometry as geo_utils
from omni.isaac.synthetic_utils import SyntheticDataHelper
import omni.isaac.core.utils.prims as prim_utils


class ObjectDetectionDatasetGenerator:
    def __init__(self, output_dir="./object_detection_dataset"):
        self.output_dir = output_dir
        self.world = World(stage_units_in_meters=1.0)
        self.object_types = ["Cube", "Cylinder", "Sphere"]
        self.setup_scene()
        self.setup_camera()
        self.setup_synthetic_data()
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "labels"), exist_ok=True)
        self.annotations = []

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

    def setup_camera(self):
        """Setup the camera for data capture"""
        self.camera_path = "/World/Camera"
        self.camera_prim = prim_utils.create_prim(
            self.camera_path,
            "Camera",
            position=[0.0, 0.0, 2.0],  # Higher for overhead view
            orientation=[0.0, 0.0, 0.0, 1.0]
        )
        
        # Set camera properties
        camera_prim = self.camera_prim
        camera_prim.GetAttribute("focallength").Set(24)
        camera_prim.GetAttribute("clippingRange").Set(Gf.Vec2f(0.1, 10000))
        camera_prim.GetAttribute("horizontalAperture").Set(20.955)
        camera_prim.GetAttribute("verticalAperture").Set(15.2908)
        
        # Set viewport resolution
        self.resolution = (640, 480)

    def setup_synthetic_data(self):
        """Setup synthetic data capture"""
        self.viewport_name = "Viewport"
        self.sd_helper = SyntheticDataHelper(
            viewport_name=self.viewport_name,
            resolution=self.resolution
        )

    def randomize_environment(self, frame_idx):
        """Randomize the environment for the frame"""
        # Clear previous objects
        for prim_path in [p for p in self.world.scene.objects if "RandomObject" in p.name]:
            self.world.scene.remove_object(prim_path.name)
        
        # Add random objects
        num_objects = random.randint(3, 8)
        frame_annotations = []
        
        for i in range(num_objects):
            # Random position
            x = random.uniform(-3, 3)
            y = random.uniform(-3, 3)
            z = random.uniform(0.2, 1.5)  # Objects above ground
            
            # Random object type
            obj_type = random.choice(self.object_types)
            
            # Random color
            color = [random.random(), random.random(), random.random()]
            
            # Create object name with type for later reference
            obj_name = f"/World/RandomObject_{frame_idx}_{i}_{obj_type}"
            
            if obj_type == "Cube":
                geo_utils.create_box(obj_name, position=[x, y, z], size=0.3, color=color)
            elif obj_type == "Cylinder":
                geo_utils.create_cylinder(obj_name, position=[x, y, z], radius=0.15, height=0.3, color=color)
            elif obj_type == "Sphere":
                geo_utils.create_sphere(obj_name, position=[x, y, z], radius=0.15, color=color)
            
            # Store annotation data
            frame_annotations.append({
                "name": obj_name,
                "type": obj_type,
                "position": {"x": x, "y": y, "z": z},
                "color": color
            })
        
        return frame_annotations

    def capture_frame(self, frame_idx):
        """Capture a frame of RGB data with annotations"""
        # Randomize environment
        annotations = self.randomize_environment(frame_idx)
        
        # Step simulation to update scene
        self.world.step(render=True)
        
        try:
            # Capture RGB image
            rgb_data = self.sd_helper.get_rgb()
            
            # Save RGB image
            img = Image.fromarray(rgb_data)
            img_path = os.path.join(self.output_dir, "images", f"frame_{frame_idx:06d}.png")
            img.save(img_path)
            
            # Save annotations
            annotation_path = os.path.join(self.output_dir, "labels", f"frame_{frame_idx:06d}.json")
            annotation_data = {
                "frame_id": frame_idx,
                "image_path": img_path,
                "objects": annotations,
                "camera_params": {
                    "resolution": self.resolution,
                    "focal_length": 24,
                    "horizontal_aperture": 20.955,
                    "vertical_aperture": 15.2908
                }
            }
            
            with open(annotation_path, 'w') as f:
                json.dump(annotation_data, f, indent=2)
            
            print(f"Saved frame {frame_idx}")
            return annotation_data
            
        except Exception as e:
            print(f"Error capturing frame {frame_idx}: {e}")
            return None

    def generate_dataset(self, num_frames=100):
        """Generate the complete dataset"""
        print(f"Generating {num_frames} frames for object detection dataset...")
        
        for i in range(num_frames):
            self.capture_frame(i)
            
            if (i + 1) % 10 == 0:
                print(f"Progress: {i + 1}/{num_frames} frames completed")
        
        print(f"Dataset generation complete! Output saved to {self.output_dir}")
        
        # Save dataset metadata
        metadata = {
            "total_frames": num_frames,
            "frame_resolution": self.resolution,
            "object_types": self.object_types,
            "date_created": str(self.world.current_time)
        }
        
        with open(os.path.join(self.output_dir, "metadata.json"), 'w') as f:
            json.dump(metadata, f, indent=2)

    def close(self):
        """Close the simulation"""
        self.world.clear()
        simulation_app.close()


def main():
    generator = ObjectDetectionDatasetGenerator(output_dir="./object_detection_dataset")
    generator.generate_dataset(num_frames=50)  # Generate 50 frames
    generator.close()


if __name__ == "__main__":
    main()
```

## Example 2: Semantic Segmentation Dataset

This example generates data for training semantic segmentation models, with RGB images and corresponding segmentation masks.

```python
# semantic_segmentation.py
import omni
from omni.isaac.kit import SimulationApp
import numpy as np
import random
import os
import json
from PIL import Image
import omni.replicator.core as rep
from pxr import Gf

# Initialize Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils import stage, light as light_utils, geometry as geo_utils
from omni.isaac.synthetic_utils import SyntheticDataHelper
from omni.isaac.core.prims import GeometryPrim
import omni.isaac.core.utils.prims as prim_utils


class SemanticSegmentationDatasetGenerator:
    def __init__(self, output_dir="./semantic_segmentation_dataset"):
        self.output_dir = output_dir
        self.world = World(stage_units_in_meters=1.0)
        
        # Define semantic classes and their colors
        self.class_definitions = {
            0: {"name": "background", "color": [0, 0, 0]},      # Black
            1: {"name": "robot", "color": [255, 0, 0]},        # Red
            2: {"name": "obstacle", "color": [0, 255, 0]},     # Green
            3: {"name": "ground", "color": [0, 0, 255]},       # Blue
            4: {"name": "wall", "color": [255, 255, 0]},       # Yellow
        }
        
        self.setup_scene()
        self.setup_camera()
        self.setup_synthetic_data()
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "masks"), exist_ok=True)

    def setup_scene(self):
        """Setup the basic scene"""
        # Add ground plane
        geo_utils.create_ground_plane("/World/GroundPlane", "x", 10, 0, 0.5, False)
        
        # Add dome light
        light_utils.create_dome_light("/World/DomeLight", color=[0.1, 0.1, 0.1], intensity=3000)

    def setup_camera(self):
        """Setup the camera for data capture"""
        self.camera_path = "/World/Camera"
        self.camera_prim = prim_utils.create_prim(
            self.camera_path,
            "Camera",
            position=[2.0, 0.0, 2.0],  # Looking down at the scene
            orientation=[-0.2588, 0.0, 0.0, 0.9659]  # 30-degree downward angle
        )
        
        # Set camera properties
        camera_prim = self.camera_prim
        camera_prim.GetAttribute("focallength").Set(24)
        camera_prim.GetAttribute("clippingRange").Set(Gf.Vec2f(0.1, 10000))
        camera_prim.GetAttribute("horizontalAperture").Set(20.955)
        camera_prim.GetAttribute("verticalAperture").Set(15.2908)
        
        self.resolution = (640, 480)

    def setup_synthetic_data(self):
        """Setup synthetic data capture"""
        self.viewport_name = "Viewport"
        self.sd_helper = SyntheticDataHelper(
            viewport_name=self.viewport_name,
            resolution=self.resolution
        )

    def randomize_environment(self, frame_idx):
        """Randomize the environment for segmentation"""
        # Clear previous objects
        for prim_path in [p for p in self.world.scene.objects if p.name != "GroundPlane"]:
            if "GroundPlane" not in p.name:
                self.world.scene.remove_object(p.name)
        
        # Add a robot-like object
        robot_color = self.class_definitions[1]["color"]
        geo_utils.create_cylinder(
            f"/World/Robot_{frame_idx}",
            position=[0, 0, 0.5],
            radius=0.3,
            height=1.0,
            color=[c/255.0 for c in robot_color]  # Normalize for Isaac Sim
        )
        
        # Add obstacles
        num_obstacles = random.randint(3, 6)
        for i in range(num_obstacles):
            pos_x = random.uniform(-4, 4)
            pos_y = random.uniform(-4, 4)
            pos_z = 0.3  # Height above ground
            
            obstacle_color = self.class_definitions[2]["color"]
            geo_utils.create_box(
                f"/World/Obstacle_{frame_idx}_{i}",
                position=[pos_x, pos_y, pos_z],
                size=0.5,
                color=[c/255.0 for c in obstacle_color]
            )

    def capture_frame(self, frame_idx):
        """Capture a frame with RGB and segmentation data"""
        # Randomize environment
        self.randomize_environment(frame_idx)
        
        # Step simulation
        self.world.step(render=True)
        
        try:
            # Capture RGB image
            rgb_data = self.sd_helper.get_rgb()
            
            # Capture semantic segmentation
            sem_data = self.sd_helper.get_semantic_segmentation()
            
            # Save RGB image
            rgb_img = Image.fromarray(rgb_data)
            rgb_path = os.path.join(self.output_dir, "images", f"rgb_{frame_idx:06d}.png")
            rgb_img.save(rgb_path)
            
            # Save segmentation mask
            seg_img = Image.fromarray(sem_data.astype(np.uint8), mode='P')
            # Add palette for visualization
            palette = []
            for i in range(256):
                if i < len(self.class_definitions):
                    color = self.class_definitions[i]["color"]
                    palette.extend(color)
                else:
                    palette.extend([0, 0, 0])  # Default to black
            
            seg_img.putpalette(palette)
            seg_path = os.path.join(self.output_dir, "masks", f"mask_{frame_idx:06d}.png")
            seg_img.save(seg_path)
            
            # Save class mapping
            class_mapping = {i: self.class_definitions[i]["name"] 
                            for i in range(len(self.class_definitions))}
            
            mapping_path = os.path.join(self.output_dir, "class_mapping.json")
            if not os.path.exists(mapping_path):
                with open(mapping_path, 'w') as f:
                    json.dump(class_mapping, f, indent=2)
            
            print(f"Saved frame {frame_idx}")
            return True
            
        except Exception as e:
            print(f"Error capturing frame {frame_idx}: {e}")
            return False

    def generate_dataset(self, num_frames=100):
        """Generate the complete segmentation dataset"""
        print(f"Generating {num_frames} frames for semantic segmentation dataset...")
        
        for i in range(num_frames):
            self.capture_frame(i)
            
            if (i + 1) % 10 == 0:
                print(f"Progress: {i + 1}/{num_frames} frames completed")
        
        print(f"Dataset generation complete! Output saved to {self.output_dir}")

    def close(self):
        """Close the simulation"""
        self.world.clear()
        simulation_app.close()


def main():
    generator = SemanticSegmentationDatasetGenerator(output_dir="./semantic_segmentation_dataset")
    generator.generate_dataset(num_frames=50)
    generator.close()


if __name__ == "__main__":
    main()
```

## Example 3: Multi-Modal Dataset with Depth and RGB

This example generates a multi-modal dataset with RGB images, depth maps, and semantic segmentation for training complex perception systems.

```python
# multi_modal_dataset.py
import omni
from omni.isaac.kit import SimulationApp
import numpy as np
import random
import os
import json
from PIL import Image
from scipy.spatial.transform import Rotation as R
import omni.replicator.core as rep
from pxr import Gf

# Initialize Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils import stage, light as light_utils, geometry as geo_utils
from omni.isaac.synthetic_utils import SyntheticDataHelper
import omni.isaac.core.utils.prims as prim_utils


class MultiModalDatasetGenerator:
    def __init__(self, output_dir="./multi_modal_dataset"):
        self.output_dir = output_dir
        self.world = World(stage_units_in_meters=1.0)
        
        # Define classes for semantic segmentation
        self.class_definitions = {
            0: {"name": "background", "color": [0, 0, 0]},
            1: {"name": "robot", "color": [255, 0, 0]},
            2: {"name": "obstacle", "color": [0, 255, 0]},
            3: {"name": "ground", "color": [0, 0, 255]},
        }
        
        self.setup_scene()
        self.setup_camera()
        self.setup_synthetic_data()
        
        # Create output directories
        for subdir in ["rgb", "depth", "segmentation", "metadata"]:
            os.makedirs(os.path.join(output_dir, subdir), exist_ok=True)

    def setup_scene(self):
        """Setup the scene with various objects"""
        # Add ground plane with texture
        geo_utils.create_ground_plane("/World/GroundPlane", "x", 10, 0, 0.5, False)
        
        # Add dome light
        light_utils.create_dome_light("/World/DomeLight", color=[0.1, 0.1, 0.1], intensity=3000)
        
        # Add directional light
        light_utils.create_directional_light("/World/DirectionalLight", 
                                           color=[1.0, 1.0, 1.0], 
                                           intensity=2000, 
                                           position=[5, 5, 10],
                                           direction=[-0.5, -0.5, -1])

    def setup_camera(self):
        """Setup the camera for multi-modal capture"""
        self.camera_path = "/World/Camera"
        self.camera_prim = prim_utils.create_prim(
            self.camera_path,
            "Camera",
            position=[2.0, 0.0, 2.0],
            orientation=[-0.2588, 0.0, 0.0, 0.9659]  # 30-degree downward angle
        )
        
        # Set camera properties
        camera_prim = self.camera_prim
        camera_prim.GetAttribute("focallength").Set(24)
        camera_prim.GetAttribute("clippingRange").Set(Gf.Vec2f(0.1, 10000))
        camera_prim.GetAttribute("horizontalAperture").Set(20.955)
        camera_prim.GetAttribute("verticalAperture").Set(15.2908)
        
        self.resolution = (640, 480)

    def setup_synthetic_data(self):
        """Setup synthetic data capture"""
        self.viewport_name = "Viewport"
        self.sd_helper = SyntheticDataHelper(
            viewport_name=self.viewport_name,
            resolution=self.resolution
        )

    def randomize_environment(self, frame_idx):
        """Randomize the environment with multiple object types"""
        # Clear previous objects
        for prim_path in [p for p in self.world.scene.objects if p.name.startswith("/World/RandomObject")]:
            self.world.scene.remove_object(prim_path.name)
        
        # Add a robot at the center
        robot_color = self.class_definitions[1]["color"]
        geo_utils.create_cylinder(
            f"/World/RandomObject_robot_{frame_idx}",
            position=[0, 0, 0.5],
            radius=0.3,
            height=1.0,
            color=[c/255.0 for c in robot_color]
        )
        
        # Add various obstacles
        num_obstacles = random.randint(5, 10)
        for i in range(num_obstacles):
            # Random position (avoid center where robot is)
            while True:
                pos_x = random.uniform(-4, 4)
                pos_y = random.uniform(-4, 4)
                # Ensure not too close to robot
                if np.sqrt(pos_x**2 + pos_y**2) > 1.0:
                    break
            
            pos_z = 0.3  # Height above ground
            
            obj_type_idx = random.choice([2])  # Only obstacle for now
            obj_color = self.class_definitions[obj_type_idx]["color"]
            
            # Random object shape
            obj_shape = random.choice(["box", "cylinder", "sphere"])
            obj_name = f"/World/RandomObject_{obj_type_idx}_{frame_idx}_{i}"
            
            if obj_shape == "box":
                geo_utils.create_box(
                    obj_name,
                    position=[pos_x, pos_y, pos_z],
                    size=0.4,
                    color=[c/255.0 for c in obj_color]
                )
            elif obj_shape == "cylinder":
                geo_utils.create_cylinder(
                    obj_name,
                    position=[pos_x, pos_y, pos_z],
                    radius=0.2,
                    height=0.6,
                    color=[c/255.0 for c in obj_color]
                )
            elif obj_shape == "sphere":
                geo_utils.create_sphere(
                    obj_name,
                    position=[pos_x, pos_y, pos_z],
                    radius=0.25,
                    color=[c/255.0 for c in obj_color]
                )

    def capture_frame(self, frame_idx):
        """Capture multi-modal data for the frame"""
        # Randomize environment
        self.randomize_environment(frame_idx)
        
        # Step simulation
        self.world.step(render=True)
        
        try:
            # Capture different data types
            rgb_data = self.sd_helper.get_rgb()
            depth_data = self.sd_helper.get_depth()
            seg_data = self.sd_helper.get_semantic_segmentation()
            
            # Save RGB image
            rgb_img = Image.fromarray(rgb_data)
            rgb_path = os.path.join(self.output_dir, "rgb", f"rgb_{frame_idx:06d}.png")
            rgb_img.save(rgb_path)
            
            # Save depth map
            depth_path = os.path.join(self.output_dir, "depth", f"depth_{frame_idx:06d}.npy")
            np.save(depth_path, depth_data)
            
            # Save segmentation
            seg_img = Image.fromarray(seg_data.astype(np.uint8), mode='P')
            # Add class palette for visualization
            palette = []
            for i in range(256):
                if i < len(self.class_definitions):
                    color = self.class_definitions[i]["color"]
                    palette.extend(color)
                else:
                    palette.extend([0, 0, 0])
            
            seg_img.putpalette(palette)
            seg_path = os.path.join(self.output_dir, "segmentation", f"seg_{frame_idx:06d}.png")
            seg_img.save(seg_path)
            
            # Save metadata
            metadata = {
                "frame_id": frame_idx,
                "timestamp": self.world.current_time,
                "camera_pos": [2.0, 0.0, 2.0],  # Fixed for this example
                "camera_orientation": [-0.2588, 0.0, 0.0, 0.9659],
                "resolution": self.resolution,
                "class_definitions": self.class_definitions,
                "data_paths": {
                    "rgb": rgb_path,
                    "depth": depth_path,
                    "segmentation": seg_path
                }
            }
            
            meta_path = os.path.join(self.output_dir, "metadata", f"meta_{frame_idx:06d}.json")
            with open(meta_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            print(f"Saved multi-modal frame {frame_idx}")
            return True
            
        except Exception as e:
            print(f"Error capturing frame {frame_idx}: {e}")
            return False

    def generate_dataset(self, num_frames=100):
        """Generate the complete multi-modal dataset"""
        print(f"Generating {num_frames} frames for multi-modal dataset...")
        
        for i in range(num_frames):
            success = self.capture_frame(i)
            if not success:
                print(f"Failed to capture frame {i}")
            
            if (i + 1) % 10 == 0:
                print(f"Progress: {i + 1}/{num_frames} frames completed")
        
        # Save dataset info
        dataset_info = {
            "total_frames": num_frames,
            "modalities": ["rgb", "depth", "segmentation"],
            "resolution": self.resolution,
            "class_definitions": self.class_definitions,
            "date_created": str(self.world.current_time),
            "output_dir": self.output_dir
        }
        
        info_path = os.path.join(self.output_dir, "dataset_info.json")
        with open(info_path, 'w') as f:
            json.dump(dataset_info, f, indent=2)
        
        print(f"Multi-modal dataset generation complete! Output saved to {self.output_dir}")

    def close(self):
        """Close the simulation"""
        self.world.clear()
        simulation_app.close()


def main():
    generator = MultiModalDatasetGenerator(output_dir="./multi_modal_dataset")
    generator.generate_dataset(num_frames=50)
    generator.close()


if __name__ == "__main__":
    main()
```

## Example 4: Domain Randomization for Robust Training

This example demonstrates how to use domain randomization to make synthetic data more effective for sim-to-real transfer.

```python
# domain_randomization.py
import omni
from omni.isaac.kit import SimulationApp
import numpy as np
import random
import os
import json
from PIL import Image
import colorsys
import omni.replicator.core as rep
from pxr import Gf, Sdf, UsdShade

# Initialize Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils import stage, light as light_utils, geometry as geo_utils
from omni.isaac.synthetic_utils import SyntheticDataHelper
import omni.isaac.core.utils.prims as prim_utils
import omni.replicator.isaac as dr


class DomainRandomizationGenerator:
    def __init__(self, output_dir="./domain_randomization_dataset"):
        self.output_dir = output_dir
        self.world = World(stage_units_in_meters=1.0)
        self.setup_scene()
        self.setup_camera()
        self.setup_synthetic_data()
        self.setup_domain_randomization()
        
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "labels"), exist_ok=True)

    def setup_scene(self):
        """Setup scene with a simple indoor environment"""
        # Add ground plane
        geo_utils.create_ground_plane("/World/GroundPlane", "x", 10, 0, 0.5, False)
        
        # Add walls
        geo_utils.create_box("/World/Wall1", position=[0, -5, 1], size=[10, 0.2, 2], color=[0.5, 0.5, 0.5])
        geo_utils.create_box("/World/Wall2", position=[0, 5, 1], size=[10, 0.2, 2], color=[0.5, 0.5, 0.5])
        geo_utils.create_box("/World/Wall3", position=[-5, 0, 1], size=[0.2, 10, 2], color=[0.5, 0.5, 0.5])
        geo_utils.create_box("/World/Wall4", position=[5, 0, 1], size=[0.2, 10, 2], color=[0.5, 0.5, 0.5])

    def setup_camera(self):
        """Setup the camera for data capture"""
        self.camera_path = "/World/Camera"
        self.camera_prim = prim_utils.create_prim(
            self.camera_path,
            "Camera",
            position=[0.0, 0.0, 2.0],
            orientation=[0.0, 0.0, 0.0, 1.0]
        )
        
        # Set camera properties
        camera_prim = self.camera_prim
        camera_prim.GetAttribute("focallength").Set(24)
        camera_prim.GetAttribute("clippingRange").Set(Gf.Vec2f(0.1, 10000))
        camera_prim.GetAttribute("horizontalAperture").Set(20.955)
        camera_prim.GetAttribute("verticalAperture").Set(15.2908)
        
        self.resolution = (640, 480)

    def setup_synthetic_data(self):
        """Setup synthetic data capture"""
        self.viewport_name = "Viewport"
        self.sd_helper = SyntheticDataHelper(
            viewport_name=self.viewport_name,
            resolution=self.resolution
        )

    def setup_domain_randomization(self):
        """Setup domain randomization configuration"""
        # This is where we'd set up replicator for domain randomization
        # For this example, we'll implement it manually
        pass

    def randomize_environment(self, frame_idx):
        """Apply domain randomization to the environment"""
        # Randomize lighting
        self.randomize_lighting(frame_idx)
        
        # Randomize material properties
        self.randomize_materials(frame_idx)
        
        # Randomize object positions
        self.randomize_objects(frame_idx)
        
        # Randomize camera parameters (subtle variations)
        self.randomize_camera(frame_idx)

    def randomize_lighting(self, frame_idx):
        """Randomize lighting conditions"""
        # Randomize dome light
        dome_light = self.world.scene.get_object("/World/DomeLight")
        intensity = random.uniform(500, 5000)
        color_temp = random.uniform(4000, 8000)  # Color temperature in Kelvin
        
        # Convert color temperature to RGB approximation
        rgb_color = self.color_temperature_to_rgb(color_temp)
        
        dome_light.set_attribute("inputs:intensity", intensity)
        dome_light.set_attribute("inputs:color", rgb_color)
        
        # Randomize directional light if exists
        if self.world.scene.get_object("/World/DirectionalLight"):
            dir_light = self.world.scene.get_object("/World/DirectionalLight")
            dir_light.set_attribute("inputs:intensity", random.uniform(1000, 3000))
            
            # Random direction
            x_dir = random.uniform(-1, 1)
            y_dir = random.uniform(-1, 1)
            z_dir = random.uniform(-1, 0)  # Usually pointing down
            dir_light.set_attribute("inputs:direction", [x_dir, y_dir, z_dir])

    def randomize_materials(self, frame_idx):
        """Randomize material properties"""
        # For this example, we'll modify the ground plane material
        ground = self.world.scene.get_object("/World/GroundPlane")
        
        # Randomize ground color
        hue = random.random()
        saturation = random.uniform(0.1, 0.8)
        value = random.uniform(0.3, 0.9)
        
        rgb = colorsys.hsv_to_rgb(hue, saturation, value)
        ground.set_attribute("inputs:diffuse_tint", [rgb[0], rgb[1], rgb[2], 1.0])
        
        # Randomize roughness
        roughness = random.uniform(0.1, 0.9)
        ground.set_attribute("inputs:roughness", roughness)

    def randomize_objects(self, frame_idx):
        """Randomize object properties and positions"""
        # Remove previously random objects
        for prim_path in [p for p in self.world.scene.objects 
                         if p.name.startswith("/World/DynamicObject")]:
            self.world.scene.remove_object(prim_path.name)
        
        # Add new random objects
        num_objects = random.randint(3, 8)
        for i in range(num_objects):
            # Random position
            x = random.uniform(-4, 4)
            y = random.uniform(-4, 4)
            z = random.uniform(0.2, 2.0)  # Height above ground
            
            # Random object type and properties
            obj_type = random.choice(["Box", "Cylinder", "Sphere"])
            size = random.uniform(0.2, 0.6)
            color = [random.random(), random.random(), random.random()]
            
            obj_name = f"/World/DynamicObject_{frame_idx}_{i}"
            
            if obj_type == "Box":
                geo_utils.create_box(obj_name, position=[x, y, z], size=size, color=color)
            elif obj_type == "Cylinder":
                geo_utils.create_cylinder(obj_name, position=[x, y, z], 
                                        radius=size/2, height=size, color=color)
            elif obj_type == "Sphere":
                geo_utils.create_sphere(obj_name, position=[x, y, z], radius=size/2, color=color)

    def randomize_camera(self, frame_idx):
        """Randomize subtle camera parameters"""
        # Randomize camera position within bounds
        cam_pos_x = random.uniform(-0.5, 0.5)
        cam_pos_y = random.uniform(-0.5, 0.5)
        cam_pos_z = random.uniform(1.8, 2.2)
        
        # Update camera position
        self.camera_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(cam_pos_x, cam_pos_y, cam_pos_z))

    def color_temperature_to_rgb(self, color_temp):
        """Convert color temperature in Kelvin to RGB"""
        temp = color_temp / 100
        
        # Red
        if temp <= 66:
            red = 255
        else:
            red = temp - 60
            red = 329.698727446 * (red ** -0.1332047592)
            red = max(0, min(255, red))
        
        # Green
        if temp <= 66:
            green = temp
            green = 99.4708025861 * np.log(green) - 161.1195681661
        else:
            green = temp - 60
            green = 288.1221695283 * (green ** -0.0755148492)
        
        green = max(0, min(255, green))
        
        # Blue
        if temp >= 66:
            blue = 255
        elif temp <= 19:
            blue = 0
        else:
            blue = temp - 10
            blue = 138.5177312231 * np.log(blue) - 305.0447927307
            blue = max(0, min(255, blue))
        
        return [red/255.0, green/255.0, blue/255.0]

    def capture_frame(self, frame_idx):
        """Capture a frame with domain randomization applied"""
        # Apply domain randomization
        self.randomize_environment(frame_idx)
        
        # Step simulation
        self.world.step(render=True)
        
        try:
            # Capture RGB image
            rgb_data = self.sd_helper.get_rgb()
            
            # Save image
            img = Image.fromarray(rgb_data)
            img_path = os.path.join(self.output_dir, "images", f"frame_{frame_idx:06d}.png")
            img.save(img_path)
            
            # Save frame info
            frame_info = {
                "frame_id": frame_idx,
                "image_path": img_path,
                "camera_pos": [self.camera_prim.GetAttribute("xformOp:translate").Get()],
                "domain_randomization_params": {
                    "light_intensity": random.uniform(500, 5000),
                    "light_color": self.color_temperature_to_rgb(random.uniform(4000, 8000)),
                    "num_objects": random.randint(3, 8)
                }
            }
            
            info_path = os.path.join(self.output_dir, "labels", f"frame_{frame_idx:06d}.json")
            with open(info_path, 'w') as f:
                json.dump(frame_info, f, indent=2)
            
            print(f"Saved domain randomization frame {frame_idx}")
            return True
            
        except Exception as e:
            print(f"Error capturing frame {frame_idx}: {e}")
            return False

    def generate_dataset(self, num_frames=100):
        """Generate the domain randomization dataset"""
        print(f"Generating {num_frames} frames with domain randomization...")
        
        for i in range(num_frames):
            success = self.capture_frame(i)
            if not success:
                print(f"Failed to capture frame {i}")
            
            if (i + 1) % 10 == 0:
                print(f"Progress: {i + 1}/{num_frames} frames completed")
        
        print(f"Domain randomization dataset generation complete! Output saved to {self.output_dir}")

    def close(self):
        """Close the simulation"""
        self.world.clear()
        simulation_app.close()


def main():
    generator = DomainRandomizationGenerator(output_dir="./domain_randomization_dataset")
    generator.generate_dataset(num_frames=50)
    generator.close()


if __name__ == "__main__":
    main()
```

## Example 5: Running the Examples

Here's a script to run the examples and monitor their execution:

```bash
#!/bin/bash
# run_synthetic_examples.sh

echo "Running Isaac Sim Synthetic Data Generation Examples"

# Create output directories
mkdir -p synthetic_examples_output

# Run basic object detection example
echo "Running basic object detection example..."
python3 basic_object_detection.py
mv object_detection_dataset synthetic_examples_output/

# Run semantic segmentation example
echo "Running semantic segmentation example..."
python3 semantic_segmentation.py
mv semantic_segmentation_dataset synthetic_examples_output/

# Run multi-modal example
echo "Running multi-modal example..."
python3 multi_modal_dataset.py
mv multi_modal_dataset synthetic_examples_output/

# Run domain randomization example
echo "Running domain randomization example..."
python3 domain_randomization.py
mv domain_randomization_dataset synthetic_examples_output/

echo "All synthetic data generation examples completed!"
echo "Output datasets are in synthetic_examples_output/ directory"
```

## Best Practices for Synthetic Data Generation

### 1. Quality Assurance
- Implement automated validation checks for each generated frame
- Verify that annotations match the generated scenes
- Check for rendering artifacts or anomalies

### 2. Performance Optimization
- Use appropriate scene complexity for your hardware
- Implement efficient data storage and retrieval
- Consider batch processing for large datasets

### 3. Diversity and Coverage
- Ensure your dataset covers the full range of target scenarios
- Include edge cases and challenging conditions
- Apply domain randomization appropriately

### 4. Documentation and Metadata
- Maintain detailed metadata for each dataset
- Document the generation process and parameters
- Include information about potential limitations or biases

## Troubleshooting Common Issues

### Performance Issues
- **Slow rendering**: Reduce scene complexity or resolution
- **Memory exhaustion**: Process frames in smaller batches
- **GPU overload**: Lower the scene complexity or processing frequency

### Quality Issues
- **Inconsistent lighting**: Ensure proper domain randomization ranges
- **Artifacts**: Verify material properties and sensor configurations
- **Poor annotations**: Check that synthetic data tools are properly configured

### Integration Issues
- **Data format compatibility**: Ensure generated data matches training pipeline requirements
- **Coordinate system mismatches**: Verify frame conventions across the pipeline
- **Timing issues**: Ensure proper synchronization between sensors

This collection of examples provides a comprehensive starting point for synthetic data generation in Isaac Sim, covering basic to advanced techniques for different AI-robotics applications.