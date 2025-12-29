---
sidebar_position: 5
---

# Chapter 11.5: Domain Randomization Techniques for Isaac Sim

## Overview

Domain randomization is a critical technique for achieving successful sim-to-real transfer in robotics. This document provides detailed guidance on implementing domain randomization specifically within NVIDIA Isaac Sim, covering both visual and physical randomization techniques that are essential for creating robust AI models that can operate effectively in the real world.

## Isaac Sim Domain Randomization Framework

Isaac Sim provides a comprehensive domain randomization framework that can be applied across multiple dimensions of the simulation. The framework allows for both manual implementation and use of Isaac Sim's built-in domain randomization tools.

### Key Concepts

- **Visual Randomization**: Randomizing appearance properties like lighting, textures, colors, and camera parameters
- **Physical Randomization**: Varying physical properties such as mass, friction, and damping
- **Dynamics Randomization**: Randomizing actuator dynamics, sensor noise, and control delays
- **Environmental Randomization**: Changing scene layouts, object positions, and environmental conditions

## Visual Domain Randomization

### Material and Texture Randomization

```python
# Example: Randomizing materials and textures in Isaac Sim
import omni
from omni.isaac.core.materials import OmniPBR
import carb
import random

def randomize_material_properties():
    """
    Randomize visual properties of materials in the simulation
    """
    # Get stage
    stage = omni.usd.get_context().get_stage()
    
    # Find all materials in the stage
    materials = []
    for prim in stage.TraverseAll():
        if prim.IsA(OmniPBR):
            materials.append(prim)
    
    for material_prim in materials:
        material = OmniPBR(prim_path=material_prim.GetPath().pathString)
        
        # Randomize roughness (0.0 to 1.0)
        roughness = random.uniform(0.0, 1.0)
        material.set_roughness(roughness)
        
        # Randomize metallic (0.0 to 1.0)
        metallic = random.uniform(0.0, 1.0)
        material.set_metallic(metallic)
        
        # Randomize color
        color = carb.Float3(random.random(), random.random(), random.random())
        material.set_color(color)
        
        # Randomize specular (0.0 to 1.0)
        specular = random.uniform(0.0, 1.0)
        material.set_specular(specular)

# Apply randomization at specific intervals during training
def apply_visual_randomization(reset_counter, interval=10):
    if reset_counter % interval == 0:
        randomize_material_properties()
```

### Lighting Randomization

```python
# Example: Randomizing lighting conditions in Isaac Sim
from pxr import UsdLux, Gf
import carb

def randomize_lighting_conditions():
    """
    Randomize lighting in the Isaac Sim environment
    """
    stage = omni.usd.get_context().get_stage()
    
    # Find all lights in the scene
    lights = [prim for prim in stage.TraverseAll() if prim.IsA(UsdLux.LightBase)]
    
    for light in lights:
        # Randomize intensity (100 to 1000)
        intensity = random.uniform(100, 1000)
        light.GetAttribute("inputs:intensity").Set(intensity)
        
        # Randomize color temperature (warm to cool white)
        color = carb.Float3(
            random.uniform(0.8, 1.0),  # Red
            random.uniform(0.8, 1.0),  # Green
            random.uniform(0.8, 1.0)   # Blue
        )
        light.GetAttribute("inputs:color").Set(color)
        
        # Randomize position (with constraints around original position)
        current_pos_attr = light.GetAttribute("xformOp:translate")
        if current_pos_attr:
            current_pos = current_pos_attr.Get()
            new_pos = [
                current_pos[0] + random.uniform(-1.0, 1.0),
                current_pos[1] + random.uniform(-1.0, 1.0),
                current_pos[2] + random.uniform(-1.0, 1.0)
            ]
            light.GetAttribute("xformOp:translate").Set(carb.Double3(*new_pos))
```

### Camera Parameter Randomization

```python
# Example: Randomizing camera parameters in Isaac Sim
def randomize_camera_parameters(camera_prim_path):
    """
    Randomize camera parameters to simulate different sensor characteristics
    """
    from omni.isaac.core.utils.prims import get_prim_at_path
    
    camera_prim = get_prim_at_path(camera_prim_path)
    
    # Randomize focal length (for depth perception differences)
    focal_length = random.uniform(15.0, 50.0)  # mm
    camera_prim.GetAttribute("focalLength").Set(focal_length)
    
    # Randomize horizontal aperture
    h_aperture = random.uniform(20.0, 36.0)  # mm
    camera_prim.GetAttribute("horizontalAperture").Set(h_aperture)
    
    # Add noise to simulate real sensor characteristics
    # This would typically be done through Isaac Sim's sensor simulation
    # capabilities or by adding post-processing effects
```

## Physical Domain Randomization

### Mass and Inertia Randomization

```python
# Example: Randomizing physical properties of robot links
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import PhysxSchema, UsdPhysics
import numpy as np

def randomize_mass_properties(robot_articulation_view):
    """
    Randomize mass properties of robot links
    """
    for i in range(robot_articulation_view.num_bodies):
        # Get current mass
        current_mass = robot_articulation_view.get_mass_matrix()[i, i, 0]
        
        # Apply randomization (±20% variation)
        mass_variation = random.uniform(0.8, 1.2)
        new_mass = current_mass * mass_variation
        
        # Apply new mass to the link
        body_handle = robot_articulation_view.body_indices[i]
        robot_articulation_view.set_mass_matrix(mass_matrix=np.array([[[new_mass]]]), indices=[body_handle])

def randomize_inertia_properties(robot_articulation_view):
    """
    Randomize inertia properties of robot links
    """
    for i in range(robot_articulation_view.num_bodies):
        # Get current inertia
        current_inertia = robot_articulation_view.get_inertia_matrix()[i, :, :, 0]
        
        # Apply randomization to diagonal elements
        inertia_diagonal = np.diag(current_inertia)
        for j in range(3):
            variation = random.uniform(0.8, 1.2)
            inertia_diagonal[j] *= variation
        
        # Create new inertia matrix with randomized diagonal
        new_inertia = np.diag(inertia_diagonal)
        
        # Apply new inertia to the link
        body_handle = robot_articulation_view.body_indices[i]
        robot_articulation_view.set_inertia_matrix(inertia_matrix=np.array([[new_inertia]]), indices=[body_handle])
```

### Friction and Damping Randomization

```python
# Example: Randomizing friction and damping parameters
def randomize_friction_and_damping(robot_articulation_view):
    """
    Randomize friction and damping parameters for robot joints
    """
    # Randomize joint friction
    for i in range(robot_articulation_view.num_dof):
        # Define friction range (0.0 to 0.5)
        friction_range = (0.0, 0.5)
        random_friction = random.uniform(*friction_range)
        
        # Apply friction to joint
        robot_articulation_view.set_joint_friction(i, random_friction)
    
    # Randomize joint damping
    for i in range(robot_articulation_view.num_dof):
        # Define damping range (0.01 to 0.1)
        damping_range = (0.01, 0.1)
        random_damping = random.uniform(*damping_range)
        
        # Apply damping to joint
        robot_articulation_view.set_joint_damping(i, random_damping)
    
    # Randomize joint stiffness
    for i in range(robot_articulation_view.num_dof):
        # Define stiffness range (0.0 to 10.0)
        stiffness_range = (0.0, 10.0)
        random_stiffness = random.uniform(*stiffness_range)
        
        # Apply stiffness to joint
        robot_articulation_view.set_joint_stiffness(i, random_stiffness)
```

## Dynamics Randomization

### Actuator Dynamics Randomization

```python
# Example: Randomizing actuator dynamics
def randomize_actuator_dynamics(robot_articulation_view):
    """
    Randomize actuator dynamics to simulate real-world variations
    """
    # Randomize actuator delays
    for i in range(robot_articulation_view.num_dof):
        # Random delay between 1ms and 10ms
        delay = random.uniform(0.001, 0.01)
        # Implementation would depend on how Isaac Sim handles actuator delays
        # This might involve custom controllers or modifying actuator properties
    
    # Randomize control precision
    for i in range(robot_articulation_view.num_dof):
        # Add noise to control signals
        control_noise_std = random.uniform(0.0, 0.02)  # 2% of control signal
        # Implementation would depend on control interface
```

### Sensor Noise Randomization

```python
# Example: Randomizing sensor noise characteristics
def randomize_sensor_noise(robot_sensors):
    """
    Randomize noise characteristics of robot sensors
    """
    for sensor in robot_sensors:
        # Randomize noise standard deviation
        noise_std_range = (0.0, 0.05)  # Adjust range based on sensor type
        noise_std = random.uniform(*noise_std_range)
        
        # Apply to sensor (implementation depends on sensor type)
        if hasattr(sensor, 'set_noise_std'):
            sensor.set_noise_std(noise_std)
        
        # Randomize bias
        bias_range = (-0.01, 0.01)
        bias = random.uniform(*bias_range)
        if hasattr(sensor, 'set_bias'):
            sensor.set_bias(bias)
```

## Isaac Sim Domain Randomization API

### Using Isaac Sim's Built-in Domain Randomization

Isaac Sim provides a high-level domain randomization API that can be used for more complex randomization scenarios:

```python
# Example: Using Isaac Sim's domain randomization extension
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.domain_randomization import DomainRandomizer

# Enable domain randomization extension
enable_extension("omni.isaac.domain_randomization")

# Create a domain randomizer instance
randomizer = DomainRandomizer()

def setup_domain_randomization():
    """
    Set up domain randomization using Isaac Sim's API
    """
    # Add randomization for material properties
    randomizer.add_randomization(
        param_name="material.roughness",
        randomization_func=lambda: random.uniform(0.0, 1.0),
        param_type="material"
    )
    
    # Add randomization for link masses
    randomizer.add_randomization(
        param_name="link.mass",
        randomization_func=lambda: random.uniform(0.8, 1.2),
        param_type="dynamics"
    )
    
    # Add randomization for joint friction
    randomizer.add_randomization(
        param_name="joint.friction",
        randomization_func=lambda: random.uniform(0.0, 0.5),
        param_type="dynamics"
    )
    
    # Apply randomization at specific intervals
    randomizer.apply_randomization(num_resets=10)

# Example of applying randomization during training
def apply_domain_randomization(reset_counter):
    """
    Apply domain randomization at specific intervals during training
    """
    if reset_counter % 10 == 0:  # Apply every 10 resets
        setup_domain_randomization()
```

## Progressive Domain Randomization

Implementing progressive randomization that increases the randomization range over time:

```python
# Example: Progressive domain randomization
def progressive_domain_randomization(training_step, max_steps):
    """
    Calculate randomization intensity based on training progress
    """
    progress = training_step / max_steps
    # Increase randomization intensity gradually
    intensity = min(progress * 1.5, 1.0)  # Max intensity of 1.0
    
    # Define base ranges for randomization
    base_roughness_range = (0.1, 0.9)
    base_mass_variation = (0.8, 1.2)
    base_friction_range = (0.0, 0.5)
    
    # Apply intensity to ranges
    roughness_range = (
        0.1 + (base_roughness_range[0] - 0.1) * (1 - intensity),
        0.9 - (0.9 - base_roughness_range[1]) * (1 - intensity)
    )
    
    mass_variation = (
        1.0 - (1.0 - base_mass_variation[0]) * intensity,
        1.0 + (base_mass_variation[1] - 1.0) * intensity
    )
    
    friction_range = (
        base_friction_range[0],
        base_friction_range[1] * intensity
    )
    
    return {
        'roughness_range': roughness_range,
        'mass_variation': mass_variation,
        'friction_range': friction_range
    }

# Apply progressive randomization
def apply_progressive_randomization(training_step, max_steps):
    """
    Apply progressively increasing domain randomization
    """
    params = progressive_domain_randomization(training_step, max_steps)
    
    # Apply visual randomization with calculated ranges
    roughness = random.uniform(*params['roughness_range'])
    # Apply to materials...
    
    # Apply physical randomization
    mass_factor = random.uniform(*params['mass_variation'])
    # Apply to masses...
    
    # Apply friction randomization
    friction = random.uniform(*params['friction_range'])
    # Apply to joints...
```

## Best Practices for Domain Randomization in Isaac Sim

### Identifying Critical Parameters

Focus randomization on parameters that significantly impact performance:

1. **Visual Parameters**: Lighting, textures, colors for perception tasks
2. **Physical Parameters**: Mass, friction, damping for control tasks
3. **Dynamics Parameters**: Actuator delays, sensor noise for real-time tasks
4. **Environmental Parameters**: Object positions, scene layout for navigation tasks

### Realistic Range Setting

Ensure randomization ranges are physically plausible:

- Don't randomize mass to negative values
- Keep friction coefficients within realistic ranges (0.0 to 1.0 for most materials)
- Ensure lighting values are within realistic intensity ranges

### Monitoring Training Stability

Track performance during randomization to avoid degrading training:

```python
def monitor_randomization_effect(training_metrics):
    """
    Monitor the effect of domain randomization on training stability
    """
    # Calculate recent performance metrics
    recent_performance = training_metrics[-10:]  # Last 10 episodes
    avg_performance = sum(recent_performance) / len(recent_performance)
    
    # Check for performance degradation
    if avg_performance < training_metrics[0] * 0.5:  # Less than 50% of initial
        print("WARNING: Significant performance degradation detected")
        print("Consider reducing randomization intensity")
```

## Troubleshooting Common Issues

### Over-Randomization

Problem: Performance degrades due to excessive randomization.
Solution: Reduce randomization range or implement progressive randomization.

```python
# Example: Detecting over-randomization
def detect_over_randomization(initial_performance, current_performance, threshold=0.3):
    """
    Detect if randomization is too aggressive
    """
    if current_performance < initial_performance * threshold:
        return True
    return False
```

### Training Instability

Problem: Training becomes unstable with high randomization.
Solution: Lower randomization frequency or reduce randomization ranges.

```python
# Example: Adaptive randomization based on training stability
def adaptive_randomization(performance_history, base_range, adjustment_factor=0.1):
    """
    Adjust randomization range based on training stability
    """
    if len(performance_history) < 20:
        return base_range
    
    # Calculate performance variance
    recent_performance = performance_history[-10:]
    variance = np.var(recent_performance)
    
    # If variance is high, reduce randomization
    if variance > np.mean(recent_performance) * 0.5:
        return (base_range[0] * (1-adjustment_factor), 
                base_range[1] * (1-adjustment_factor))
    
    # If variance is low and performance is improving, can increase randomization
    recent_trend = np.polyfit(range(len(recent_performance)), recent_performance, 1)
    if recent_trend[0] > 0:  # Positive trend
        return (min(base_range[0] * (1+adjustment_factor), 1.0), 
                min(base_range[1] * (1+adjustment_factor), 1.0))
    
    return base_range
```

## Assessment Questions

1. Explain the difference between visual and physical domain randomization in Isaac Sim.
2. How can you implement progressive domain randomization that increases over time?
3. What are the key parameters to randomize for humanoid robot control tasks?
4. How do you ensure domain randomization ranges remain physically plausible?
5. What techniques can be used to monitor the effectiveness of domain randomization?

## Implementation Checklist

- [ ] Visual properties (materials, lighting, textures) are randomized
- [ ] Physical properties (mass, friction, damping) are randomized
- [ ] Dynamics parameters (delays, noise) are randomized
- [ ] Randomization ranges are physically plausible
- [ ] Training stability is monitored during randomization
- [ ] Progressive randomization is implemented where appropriate
- [ ] Isaac Sim's domain randomization API is properly utilized