---
sidebar_position: 2
---

# Section 2.2: Creating Physical Properties for Humanoid Robots

## Overview

This section covers the process of defining and configuring physical properties for humanoid robot models in Gazebo. Students will learn how to set mass, inertia, friction, and other properties that affect the robot's behavior in simulation.

## Physical Properties Overview

For each link in a humanoid robot model, several physical properties must be defined:

- **Mass**: The amount of matter in the link
- **Inertia**: Resistance to rotational motion (inertia tensor)
- **Center of Mass**: The point where mass can be considered concentrated
- **Collision Properties**: How the link interacts with other objects
- **Visual Properties**: How the link appears (separate from physics)

## Mass Properties

### Mass Definition

Mass should be set based on the real-world equivalent of the robot part:

```xml
<link name="upper_arm">
  <inertial>
    <mass value="2.5"/>  <!-- Mass in kg -->
    <origin xyz="0 0 0.1" rpy="0 0 0"/>  <!-- Center of mass offset -->
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>  <!-- Inertia tensor -->
  </inertial>
</link>
```

### Mass Estimation Guidelines

For humanoid robots, typical mass values:
- Head: 4-5 kg
- Upper arm: 2-3 kg
- Lower arm: 1.5-2 kg
- Torso: 25-35 kg
- Upper leg: 8-10 kg
- Lower leg: 6-8 kg
- Foot: 1-2 kg

## Inertia Tensor

### Understanding Inertia

The inertia tensor describes how mass is distributed in a 3D object. For a humanoid robot link:

- `ixx`, `iyy`, `izz`: Moments of inertia about the x, y, z axes
- `ixy`, `ixz`, `iyz`: Products of inertia (usually 0 for symmetric objects)

### Calculating Inertia

For common shapes, use these formulas (where m = mass, dimensions in meters):

**Cylinder (approximation for limbs):**
- ixx = iyy = m*(3*r² + h²)/12
- izz = m*r²/2

**Box (approximation for torso):**
- ixx = m*(h² + d²)/12
- iyy = m*(w² + d²)/12
- izz = m*(w² + h²)/12

### Example Inertia Calculation

For a cylindrical upper arm (mass = 2.5kg, radius = 0.05m, height = 0.3m):
- ixx = iyy = 2.5*(3*0.05² + 0.3²)/12 = 0.019 kg·m²
- izz = 2.5*0.05²/2 = 0.003 kg·m²

```xml
<inertial>
  <mass value="2.5"/>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
  <inertia ixx="0.019" ixy="0" ixz="0" iyy="0.019" iyz="0" izz="0.003"/>
</inertial>
```

## Collision Properties

### Collision Geometry

Collision geometry defines how the robot interacts with the environment:

```xml
<link name="upper_arm">
  <collision name="collision">
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
  </collision>
</link>
```

### Collision vs Visual Geometry

Collision geometry can be different from visual geometry:
- Use simpler shapes for collision (faster computation)
- Use detailed shapes for visual (better appearance)
- Collision can be offset from visual for better physics

## Friction and Damping

### Friction Properties

Friction affects how the robot interacts with surfaces:

```xml
<collision name="collision">
  <surface>
    <friction>
      <ode>
        <mu>0.5</mu>  <!-- Coefficient of friction -->
        <mu2>0.5</mu2>  <!-- Secondary coefficient -->
      </ode>
    </friction>
  </surface>
</collision>
```

### Damping Properties

Damping reduces oscillations in joints:

```xml
<joint name="elbow_joint" type="revolute">
  <dynamics damping="0.1" friction="0.05"/>
</joint>
```

## Center of Mass Considerations

### Proper CoM Placement

For stable humanoid robots:
- Keep overall CoM within the support polygon
- Lower CoM increases stability
- Consider CoM during motion planning

### Example CoM Adjustment

```xml
<inertial>
  <mass value="10"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>  <!-- Lower CoM for stability -->
  <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.2"/>
</inertial>
```

## Validation Techniques

### Physics Validation

1. **Static Balance**: Check if the robot remains stable when standing
2. **Dynamic Response**: Verify realistic movement to applied forces
3. **Energy Conservation**: Ensure no unexpected energy gain/loss

### Tools for Validation

- Gazebo's built-in model inspector
- URDF validation tools: `check_urdf <model.urdf>`
- Physics parameter calculators

## Best Practices

1. **Realistic Values**: Base parameters on real-world robot specifications
2. **Iterative Refinement**: Start with estimates, then tune based on behavior
3. **Symmetry**: Ensure left/right limbs have matching properties
4. **Documentation**: Keep a record of how values were determined
5. **Validation**: Regularly test physics behavior against expectations

## Troubleshooting Common Issues

### Robot Falls Over
- Check mass distribution and CoM position
- Verify joint limits and constraints
- Adjust friction coefficients

### Unstable Oscillations
- Increase joint damping
- Reduce simulation time step
- Check inertia values for accuracy

### Penetration Issues
- Verify collision geometry overlaps
- Check for proper mesh scaling
- Adjust collision margins if needed

## Next Steps

After configuring physical properties for humanoid robots, proceed to [Section 2.3: Collision Detection and Response](../chapter-2/section-2-3-collision-detection) to learn about collision handling in simulation.