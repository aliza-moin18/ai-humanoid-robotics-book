---
sidebar_position: 2
---

# Section 3.2: Physics Parameters in Gazebo

## Overview

This section covers advanced physics parameter configuration in Gazebo, focusing on global physics settings, engine-specific parameters, and optimization techniques for humanoid robot simulation. Students will learn how to configure physics engines for optimal performance and stability.

## Global Physics Configuration

### World File Physics Settings

Physics parameters are typically defined in the world file:

```xml
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Physics engine configuration -->
    <physics type="dart" name="dart_physics">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      
      <!-- DART-specific parameters -->
      <dart>
        <solver>
          <solver_type>PGS</solver_type>
          <max_iterations>100</max_iterations>
          <collision_detector>fcl</collision_detector>
        </solver>
      </dart>
    </physics>
    
    <!-- Rest of world content -->
  </world>
</sdf>
```

### Physics Engine Selection

#### DART (Recommended for Humanoid Robots)
```xml
<physics type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <dart>
    <solver>
      <solver_type>PGS</solver_type>  <!-- PGS (Projected Gauss-Seidel) -->
      <max_iterations>100</max_iterations>
      <collision_detector>fcl</collision_detector>  <!-- Flexible Collision Library -->
    </solver>
  </dart>
</physics>
```

#### ODE Configuration
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <ode>
    <solver>
      <type>quick</type>  <!-- or "world" for more accuracy -->
      <iters>20</iters>    <!-- Solver iterations -->
      <sor>1.3</sor>      <!-- Successive Over-Relaxation parameter -->
    </solver>
    <constraints>
      <cfm>0.00001</cfm>   <!-- Constraint Force Mixing -->
      <erp>0.2</erp>      <!-- Error Reduction Parameter -->
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Time Step Configuration

### Time Step Selection Guidelines

The time step (`max_step_size`) significantly affects simulation stability and performance:

- **0.001s (1ms)**: Good for most humanoid robot simulations
- **0.0005s (0.5ms)**: For high-precision or fast dynamics
- **0.002s (2ms)**: For less complex simulations or performance optimization

### Real-Time Factor

The real-time factor controls simulation speed relative to wall-clock time:

```xml
<physics type="dart">
  <real_time_factor>1.0</real_time_factor>  <!-- Simulate in real-time -->
  <real_time_factor>0.5</real_time_factor>  <!-- Simulate at half speed -->
  <real_time_factor>2.0</real_time_factor>  <!-- Simulate at double speed -->
</physics>
```

## Solver Parameters

### DART Solver Configuration

For humanoid robots using DART:

```xml
<physics type="dart">
  <dart>
    <solver>
      <!-- Solver type options: PGS, Dantzig -->
      <solver_type>PGS</solver_type>
      
      <!-- Maximum iterations for convergence -->
      <max_iterations>100</max_iterations>
      
      <!-- Collision detection algorithm -->
      <collision_detector>fcl</collision_detector>
      
      <!-- Constraint violation tolerance -->
      <constraint_tolerance>1e-6</constraint_tolerance>
    </solver>
  </dart>
</physics>
```

### ODE Solver Configuration

For compatibility or specific requirements:

```xml
<physics type="ode">
  <ode>
    <solver>
      <type>quick</type>    <!-- QuickStep solver -->
      <iters>50</iters>     <!-- Iterations per step -->
      <sor>1.3</sor>        <!-- Successive Over-Relaxation -->
    </solver>
    <constraints>
      <cfm>0.00001</cfm>    <!-- Constraint Force Mixing -->
      <erp>0.2</erp>        <!-- Error Reduction Parameter -->
    </constraints>
  </ode>
</physics>
```

## Gravity Configuration

### Standard Gravity

For Earth-like gravity:

```xml
<physics type="dart">
  <gravity>0 0 -9.8</gravity>  <!-- x, y, z components in m/s² -->
</physics>
```

### Custom Gravity

For different environments:

```xml
<physics type="dart">
  <gravity>0 0 -1.62</gravity>  <!-- Moon gravity -->
  <gravity>0 0 -3.7</gravity>   <!-- Mars gravity -->
  <gravity>0 0 0</gravity>      <!-- Zero gravity -->
</physics>
```

## Advanced Physics Parameters

### Contact Parameters

Fine-tune contact behavior for realistic interactions:

```xml
<physics type="dart">
  <dart>
    <solver>
      <solver_type>PGS</solver_type>
      <max_iterations>100</max_iterations>
      <collision_detector>fcl</collision_detector>
    </solver>
  </dart>
  
  <!-- Global contact parameters -->
  <constraints>
    <contact_surface_layer>0.001</contact_surface_layer>
    <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
  </constraints>
</physics>
```

### Performance Optimization Parameters

For balancing accuracy and performance:

```xml
<physics type="dart">
  <!-- Time stepping -->
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  
  <!-- Solver settings -->
  <dart>
    <solver>
      <solver_type>PGS</solver_type>
      <max_iterations>50</max_iterations>  <!-- Lower for performance -->
      <collision_detector>bullet</collision_detector>  <!-- Faster collision detection -->
    </solver>
  </dart>
</physics>
```

## Humanoid Robot-Specific Considerations

### Stability Parameters

For stable humanoid robot simulation:

```xml
<physics type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>0.8</real_time_factor>  <!-- Slightly slower for stability -->
  <dart>
    <solver>
      <solver_type>PGS</solver_type>
      <max_iterations>100</max_iterations>  <!-- Higher for stability -->
      <collision_detector>fcl</collision_detector>
    </solver>
  </dart>
</physics>
```

### Multi-Body Dynamics

For complex articulated robots:

```xml
<physics type="dart">
  <!-- Use DART for best articulated body handling -->
  <max_step_size>0.001</max_step_size>
  <dart>
    <solver>
      <solver_type>PGS</solver_type>
      <max_iterations>120</max_iterations>  <!-- Higher for complex systems -->
      <collision_detector>fcl</collision_detector>
    </solver>
  </dart>
</physics>
```

## Physics Parameter Tuning Process

### Step-by-Step Tuning

1. **Start with defaults**: Begin with standard parameters
2. **Test basic stability**: Verify robot stands without falling
3. **Adjust solver iterations**: Increase if experiencing instability
4. **Optimize time step**: Balance between stability and performance
5. **Fine-tune contact parameters**: Adjust for realistic interactions
6. **Validate with tasks**: Test with intended robot behaviors

### Performance Monitoring

Monitor these metrics during tuning:

- **Simulation speed**: Real-time factor achieved
- **CPU usage**: Processor load during simulation
- **Stability**: Absence of jittering or unexpected movements
- **Accuracy**: Realistic response to forces and contacts

## Validation and Testing

### Physics Validation Tests

#### Stability Test
```xml
<!-- Simple test world for physics validation -->
<sdf version="1.7">
  <world name="physics_validation">
    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <model name="test_box">
      <pose>0 0 1 0 0 0</pose>
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx> <ixy>0</ixy> <ixz>0</ixz>
            <iyy>0.1</iyy> <iyz>0</iyz> <izz>0.1</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box><size>0.2 0.2 0.2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 0.2 0.2</size></box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

#### Performance Test
- Monitor simulation speed with multiple objects
- Test with complex articulated robots
- Verify consistent frame rate

## Common Physics Issues and Solutions

### Issue 1: Simulation Instability
**Symptoms**: Objects jitter, explode, or behave erratically
**Solutions**:
- Reduce time step size
- Increase solver iterations
- Verify mass and inertia values
- Check for intersecting collision geometries

### Issue 2: Slow Simulation
**Symptoms**: Simulation runs significantly slower than real-time
**Solutions**:
- Increase time step size (carefully)
- Reduce solver iterations
- Optimize collision geometries
- Simplify model complexity

### Issue 3: Unrealistic Contact Behavior
**Symptoms**: Objects penetrate or bounce unrealistically
**Solutions**:
- Adjust contact parameters (ERP, CFM)
- Verify collision geometry
- Check friction coefficients
- Ensure proper mass distribution

## Best Practices

1. **Use DART for Humanoid Robots**: DART provides superior handling of articulated systems
2. **Conservative Initial Settings**: Start with stable parameters, then optimize
3. **Validate with Simple Models**: Test physics parameters with basic objects first
4. **Document Parameter Changes**: Keep records of what works for different scenarios
5. **Balance Accuracy and Performance**: Adjust parameters based on simulation requirements
6. **Regular Testing**: Continuously validate physics behavior during development

## Next Steps

After configuring physics parameters in Gazebo, proceed to [Section 3.3: Joint and Link Configurations](../chapter-3/section-3-3-joint-link-configurations) to learn about advanced joint and link setup for humanoid robots.