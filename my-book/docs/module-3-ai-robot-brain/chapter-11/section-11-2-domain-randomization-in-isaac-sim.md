---
sidebar_position: 2
---

# Chapter 11.2: Domain Randomization in Isaac Sim

## Introduction

Domain randomization is a powerful technique for bridging the sim-to-real gap by training models on diverse simulated environments with randomized parameters. In NVIDIA Isaac Sim, domain randomization can be applied across multiple dimensions to create robust models that generalize well to real-world conditions.

This chapter explores how to implement effective domain randomization strategies using Isaac Sim's built-in tools and capabilities.

## Understanding Domain Randomization

Domain randomization operates on the principle that if a model is trained on a wide variety of conditions, it will learn to focus on the essential features needed for the task rather than on domain-specific artifacts. The key is to randomize all aspects of the simulation that might differ from the real world.

### Types of Domain Randomization

1. **Visual Randomization**: Randomizing colors, textures, lighting, and camera parameters
2. **Physical Randomization**: Varying mass, friction, damping, and other physical properties
3. **Dynamics Randomization**: Randomizing actuator dynamics, sensor noise, and control delays
4. **Environmental Randomization**: Changing scene layouts, object positions, and environmental conditions

## Isaac Sim Domain Randomization Framework

Isaac Sim provides a comprehensive domain randomization framework that can be applied at different levels:

### Randomizing Visual Properties

```python
# Example: Randomizing visual properties in Isaac Sim
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.materials import OmniPBR
import random

def randomize_materials(robot_prim_path):
    # Get all materials in the robot
    materials = get_robot_materials(robot_prim_path)
    
    for material in materials:
        # Randomize roughness (0.0 to 1.0)
        material.set_roughness(random.uniform(0.0, 1.0))
        
        # Randomize metallic value (0.0 to 1.0)
        material.set_metallic(random.uniform(0.0, 1.0))
        
        # Randomize color
        color = [random.random() for _ in range(3)]
        material.set_color(color)
        
        # Randomize specular value
        material.set_specular(random.uniform(0.1, 0.9))
```

### Randomizing Physical Parameters

```python
# Example: Randomizing physical properties of robot joints
from omni.isaac.core.utils.stage import get_current_stage
from pxr import PhysxSchema

def randomize_robot_dynamics(robot_articulation):
    # Randomize joint friction
    for joint in robot_articulation.joints:
        friction_range = (0.0, 0.5)  # Define range for friction
        random_friction = random.uniform(*friction_range)
        joint.set_joint_friction(random_friction)
        
    # Randomize link masses
    for link in robot_articulation.links:
        # Get current mass
        current_mass = link.get_mass()
        # Randomize within ±20% of original mass
        mass_variation = random.uniform(0.8, 1.2)
        new_mass = current_mass * mass_variation
        link.set_mass(new_mass)
```

## Implementing Domain Randomization in Isaac Sim

### Using Isaac Sim's Domain Randomization API

Isaac Sim provides a high-level API for domain randomization:

```python
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.domain_randomization import DomainRandomizer

# Enable domain randomization extension
enable_extension("omni.isaac.domain_randomization")

# Create a domain randomizer
randomizer = DomainRandomizer()

# Add randomization for specific parameters
randomizer.add_randomization(
    param_name="camera.noise_std",
    randomization_func=lambda: random.uniform(0.0, 0.05),
    param_type="sensor"
)

# Apply randomization at specific intervals
randomizer.apply_randomization(num_resets=10)
```

### Randomizing Lighting Conditions

```python
# Example: Randomizing lighting in Isaac Sim
import omni
from pxr import UsdLux
import carb

def randomize_lighting():
    stage = omni.usd.get_context().get_stage()
    
    # Find all lights in the scene
    lights = [prim for prim in stage.TraverseAll() if prim.IsA(UsdLux.LightBase)]
    
    for light in lights:
        # Randomize intensity
        light.GetAttribute("inputs:intensity").Set(random.uniform(100, 1000))
        
        # Randomize color temperature
        light.GetAttribute("inputs:color").Set(
            carb.Float3(random.uniform(0.5, 1.0), 
                       random.uniform(0.5, 1.0), 
                       random.uniform(0.5, 1.0))
        )
        
        # Randomize position (with constraints)
        current_pos = light.GetAttribute("xformOp:translate").Get()
        new_pos = [
            current_pos[0] + random.uniform(-0.5, 0.5),
            current_pos[1] + random.uniform(-0.5, 0.5),
            current_pos[2] + random.uniform(-0.5, 0.5)
        ]
        light.GetAttribute("xformOp:translate").Set(carb.Double3(*new_pos))
```

## Advanced Domain Randomization Techniques

### Texture Randomization

```python
# Randomizing textures using Isaac Sim's material system
def randomize_textures():
    # Randomize ground plane texture
    ground_material = get_ground_material()
    
    # Randomize texture scale
    scale_range = (0.5, 2.0)
    new_scale = random.uniform(*scale_range)
    ground_material.set_texture_scale(new_scale)
    
    # Randomize texture rotation
    rotation = random.uniform(0, 360)
    ground_material.set_texture_rotation(rotation)
```

### Dynamics Randomization

```python
# Randomizing robot dynamics parameters
def randomize_dynamics(robot_articulation_view):
    # Randomize control frequency
    control_freq = random.uniform(50, 200)  # Hz
    
    # Randomize actuator delays
    for i in range(robot_articulation_view.num_dof):
        delay = random.uniform(0.001, 0.01)  # 1-10ms delay
        robot_articulation_view.set_actuator_delay(i, delay)
    
    # Randomize sensor noise
    for sensor in robot_articulation_view.sensors:
        noise_std = random.uniform(0.0, 0.05)
        sensor.set_noise_std(noise_std)
```

## Domain Randomization Strategies

### Progressive Randomization

Start with minimal randomization and gradually increase the range as training progresses:

```python
def progressive_randomization(training_step, max_steps):
    # Calculate randomization intensity based on training progress
    progress = training_step / max_steps
    intensity = min(progress * 2, 1.0)  # Max intensity of 1.0
    
    # Apply randomization with calculated intensity
    roughness_range = (0.1 * intensity, 0.9 * intensity)
    return random.uniform(*roughness_range)
```

### Curriculum Randomization

Organize randomization in a curriculum that gradually increases complexity:

1. **Phase 1**: Randomize only visual properties
2. **Phase 2**: Add physical property randomization
3. **Phase 3**: Include dynamics randomization
4. **Phase 4**: Full domain randomization

## Evaluating Domain Randomization Effectiveness

### Metrics for Success

1. **Zero-shot transfer performance**: How well the model performs on real robots without fine-tuning
2. **Sample efficiency**: How quickly the model adapts with limited real-world data
3. **Robustness**: Performance consistency across different real-world conditions
4. **Generalization**: Performance on previously unseen scenarios

### Validation Techniques

```python
def validate_domain_randomization():
    # Test model on various randomized conditions
    performance_metrics = []
    
    for condition in test_conditions:
        # Apply condition to simulation
        apply_condition(condition)
        
        # Evaluate model performance
        performance = evaluate_model()
        performance_metrics.append(performance)
    
    # Calculate statistics
    mean_performance = sum(performance_metrics) / len(performance_metrics)
    variance = calculate_variance(performance_metrics)
    
    return mean_performance, variance
```

## Best Practices for Domain Randomization

1. **Identify Critical Parameters**: Focus randomization on parameters that significantly impact performance
2. **Realistic Ranges**: Ensure randomization ranges are physically plausible
3. **Monitor Training**: Track performance during randomization to avoid degrading training
4. **Validation**: Regularly validate randomized models in simulation before real-world testing
5. **Balance**: Balance randomization intensity to maintain training stability

## Troubleshooting Common Issues

### Over-Randomization

Problem: Performance degrades due to excessive randomization.
Solution: Reduce randomization range or implement progressive randomization.

### Training Instability

Problem: Training becomes unstable with high randomization.
Solution: Lower learning rates or reduce randomization frequency.

### Poor Real-World Performance

Problem: Model doesn't transfer well despite randomization.
Solution: Identify missing real-world factors and add them to randomization.

## Assessment Questions

1. Explain the difference between visual, physical, and dynamics randomization.
2. How does progressive randomization differ from uniform randomization?
3. What are the key parameters to randomize for humanoid robot control?
4. How can you validate that your domain randomization is effective?

## Lab Preparation

In the next lab, you will implement domain randomization for a specific robot task by:
1. Identifying key parameters to randomize for your robot
2. Implementing a domain randomization pipeline in Isaac Sim
3. Training a policy with and without domain randomization
4. Comparing the sim-to-real transfer performance of both approaches