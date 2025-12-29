---
sidebar_position: 4
---

# Section 3.4: Environmental Physics Simulation

## Overview

This section covers environmental physics simulation in Gazebo, focusing on creating realistic environments for humanoid robot simulation. Students will learn about terrain modeling, fluid dynamics, atmospheric effects, and how to configure environmental physics parameters for realistic robot-environment interactions.

## Terrain and Ground Simulation

### Flat Ground Plane

The most basic environment:

```xml
<sdf version="1.7">
  <world name="simple_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Physics properties for ground -->
    <model name="ground_plane">
      <link name="link">
        <collision name="collision">
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### Heightmap Terrain

For realistic outdoor environments:

```xml
<model name="terrain" static="true">
  <link name="terrain_link">
    <collision name="collision">
      <geometry>
        <heightmap>
          <uri>model://terrain/heightmap.png</uri>
          <size>100 100 20</size>  <!-- x, y, z dimensions in meters -->
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <heightmap>
          <uri>model://terrain/heightmap.png</uri>
          <size>100 100 20</size>
        </heightmap>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Dirt</name>
        </script>
      </material>
    </visual>
  </link>
</model>
```

### Procedural Terrain

Creating terrain programmatically:

```xml
<model name="procedural_terrain" static="true">
  <link name="terrain_link">
    <collision name="collision">
      <geometry>
        <mesh>
          <uri>model://terrain/procedural_mesh.dae</uri>
        </mesh>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>model://terrain/procedural_mesh.dae</uri>
        </mesh>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Grass</name>
        </script>
      </material>
    </visual>
  </link>
</model>
```

## Obstacle and Object Simulation

### Static Obstacles

For creating static obstacles in the environment:

```xml
<model name="table" static="true">
  <pose>2 0 0 0 0 0</pose>
  <link name="table_top">
    <inertial>
      <mass>10</mass>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 0.8 0.02</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.6 0.4 1</ambient>
        <diffuse>0.8 0.6 0.4 1</diffuse>
      </material>
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>1 0.8 0.02</size>
        </box>
      </geometry>
    </collision>
  </link>
  
  <!-- Table legs -->
  <link name="leg1">
    <pose>0.4 0.3 -0.35 0 0 0</pose>
    <visual>
      <geometry>
        <cylinder>
          <radius>0.02</radius>
          <length>0.7</length>
        </cylinder>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder>
          <radius>0.02</radius>
          <length>0.7</length>
        </cylinder>
      </geometry>
    </collision>
  </link>
</model>
```

### Dynamic Objects

For objects that can be manipulated by the robot:

```xml
<model name="movable_box">
  <pose>1 0 0.5 0 0 0</pose>
  <link name="box_link">
    <inertial>
      <mass>2.0</mass>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.2 0.2 0.2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.5 0.5 1 1</ambient>
        <diffuse>0.5 0.5 1 1</diffuse>
      </material>
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>0.2 0.2 0.2</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>
```

## Fluid Dynamics Simulation

### Simple Fluid Effects

Gazebo doesn't have full fluid simulation, but you can simulate basic effects:

```xml
<model name="water_body" static="true">
  <link name="water_surface">
    <collision name="collision">
      <geometry>
        <box>
          <size>10 10 0.1</size>
        </box>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.1</mu>  <!-- Low friction for slippery surface -->
            <mu2>0.1</mu2>
          </ode>
        </friction>
        <bounce>
          <restitution_coefficient>0.0</restitution_coefficient>  <!-- No bounce -->
        </bounce>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>10 10 0.1</size>
        </box>
      </geometry>
      <material>
        <ambient>0 0.5 0.8 0.5</ambient>
        <diffuse>0 0.5 0.8 0.5</diffuse>
        <specular>0.5 0.5 0.5 0.5</specular>
      </material>
    </visual>
  </link>
</model>
```

### Buoyancy Simulation

Simulating buoyancy effects with external plugins:

```xml
<model name="buoyant_object">
  <link name="object">
    <inertial>
      <mass>0.5</mass>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <sphere>
          <radius>0.1</radius>
        </sphere>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere>
          <radius>0.1</radius>
        </sphere>
      </geometry>
    </collision>
    
    <!-- Gazebo-specific buoyancy plugin -->
    <gazebo>
      <plugin name="buoyancy_plugin" filename="libBuoyancyPlugin.so">
        <fluid_density>1000</fluid_density>  <!-- Water density -->
        <link_name>object</link_name>
      </plugin>
    </gazebo>
  </link>
</model>
```

## Atmospheric and Environmental Effects

### Wind Simulation

Adding wind forces to the environment:

```xml
<world name="windy_world">
  <physics type="dart">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
  
  <model name="wind_generator" static="true">
    <link name="wind_link">
      <inertial>
        <mass>0.001</mass>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial>
    </link>
    
    <!-- Wind plugin -->
    <gazebo>
      <plugin name="wind_plugin" filename="libWindPlugin.so">
        <wind_direction>1 0 0</wind_direction>
        <wind_force>0.5 0 0</wind_force>
        <start_time>0</start_time>
      </plugin>
    </gazebo>
  </model>
</world>
```

### Gravity Variations

Simulating different gravitational environments:

```xml
<world name="moon_environment">
  <physics type="dart">
    <gravity>0 0 -1.62</gravity>  <!-- Moon gravity -->
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
  
  <!-- Moon surface -->
  <model name="moon_surface" static="true">
    <link name="surface">
      <collision name="collision">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
          </plane>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.3</mu>
              <mu2>0.3</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>100 100</size>
          </plane>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</world>
```

## Complex Environmental Scenarios

### Indoor Environment

Creating a room with furniture:

```xml
<sdf version="1.7">
  <world name="indoor_world">
    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Walls -->
    <model name="wall_1" static="true">
      <pose>0 5 1 0 0 0</pose>
      <link name="wall_link">
        <collision name="collision">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Furniture -->
    <model name="chair">
      <pose>1 1 0 0 0 0</pose>
      <link name="seat">
        <inertial>
          <mass>5</mass>
          <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.2"/>
        </inertial>
        <visual>
          <geometry>
            <box><size>0.4 0.4 0.05</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
          </material>
        </visual>
        <collision>
          <geometry>
            <box><size>0.4 0.4 0.05</size></box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### Outdoor Environment with Obstacles

Creating an outdoor environment with various obstacles:

```xml
<world name="outdoor_world">
  <physics type="dart">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
  
  <!-- Ground terrain -->
  <model name="terrain" static="true">
    <link name="ground">
      <collision name="collision">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
          </plane>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
  
  <!-- Obstacles -->
  <model name="obstacle_1" static="true">
    <pose>3 0 0.5 0 0 0</pose>
    <link name="obstacle_link">
      <collision>
        <geometry>
          <box><size>0.5 0.5 1</size></box>
        </geometry>
      </collision>
      <visual>
        <geometry>
          <box><size>0.5 0.5 1</size></box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
        </material>
      </visual>
    </link>
  </model>
  
  <!-- Ramp for testing locomotion -->
  <model name="ramp" static="true">
    <pose>5 0 0 0 0 0.3</pose>
    <link name="ramp_link">
      <collision>
        <geometry>
          <mesh><uri>model://ramp.dae</uri></mesh>
        </geometry>
      </collision>
      <visual>
        <geometry>
          <mesh><uri>model://ramp.dae</uri></mesh>
        </geometry>
      </visual>
    </link>
  </model>
</world>
```

## Environmental Physics Validation

### Ground Contact Validation

Testing proper ground contact:

```xml
<model name="contact_test">
  <pose>0 0 1 0 0 0</pose>
  <link name="test_object">
    <inertial>
      <mass>1.0</mass>
      <inertia ixx="0.1" iyy="0.1" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <sphere><radius>0.1</radius></sphere>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere><radius>0.1</radius></sphere>
      </geometry>
    </collision>
  </link>
</model>
```

### Friction Validation

Testing different friction coefficients:

```xml
<world name="friction_test_world">
  <physics type="dart">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
  
  <!-- Different surface materials -->
  <model name="surface_01" static="true">
    <pose>0 0 0 0 0 0</pose>
    <link name="surface">
      <collision name="collision">
        <geometry>
          <box><size>2 2 0.1</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.1</mu>  <!-- Low friction -->
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
  
  <model name="surface_08" static="true">
    <pose>3 0 0 0 0 0</pose>
    <link name="surface">
      <collision name="collision">
        <geometry>
          <box><size>2 2 0.1</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>  <!-- High friction -->
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
</world>
```

## Performance Optimization

### Simplified Collision Models

Using simplified collision geometries for performance:

```xml
<link name="complex_visual_simple_collision">
  <!-- Detailed visual model -->
  <visual name="visual">
    <geometry>
      <mesh><uri>model://robot/meshes/complex_shape.dae</uri></mesh>
    </geometry>
  </visual>
  
  <!-- Simplified collision model -->
  <collision name="collision">
    <geometry>
      <box><size>0.2 0.15 0.3</size></box>  <!-- Approximating the complex shape -->
    </geometry>
  </collision>
</link>
```

### Static Objects

Marking objects as static when they don't move:

```xml
<model name="static_environment" static="true">
  <link name="static_link">
    <!-- Static objects don't need inertial properties -->
    <visual>
      <geometry>
        <mesh><uri>model://environment/ground.dae</uri></mesh>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh><uri>model://environment/ground.dae</uri></mesh>
      </geometry>
    </collision>
  </link>
</model>
```

## Best Practices

1. **Realistic Materials**: Use appropriate friction and restitution values
2. **Static Optimization**: Mark immovable objects as static
3. **Collision Simplification**: Use simplified geometries for collision when possible
4. **Layered Environments**: Build complex environments in layers
5. **Validation Testing**: Test robot behavior in each environment type
6. **Performance Monitoring**: Monitor simulation performance with complex environments
7. **Modular Design**: Create reusable environmental components

## Troubleshooting Common Issues

### Issue 1: Objects Falling Through Ground
- **Check**: Collision geometry alignment
- **Verify**: Surface parameters (friction, restitution)
- **Adjust**: Contact parameters (ERP, CFM)

### Issue 2: Unstable Object Stacking
- **Check**: Mass and inertia values
- **Verify**: Contact parameters
- **Adjust**: Physics time step and solver settings

### Issue 3: Performance Degradation
- **Check**: Number of active objects
- **Verify**: Collision geometry complexity
- **Optimize**: Use static models where appropriate

## Next Steps

After mastering environmental physics simulation, proceed to [Chapter 4: Sensor Integration](../chapter-4/index) to learn about integrating various sensors in digital twin simulations.