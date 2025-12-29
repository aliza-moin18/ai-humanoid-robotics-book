---
sidebar_position: 3
---

# Section 2.3: Collision Detection and Response

## Overview

This section covers collision detection and response mechanisms in Gazebo, focusing on how humanoid robots interact with their environment. Students will learn about contact physics, collision algorithms, and how to configure collision responses for realistic interactions.

## Collision Detection Fundamentals

### Contact Physics

When two objects come into contact in Gazebo, several physical phenomena occur:

1. **Penetration Prevention**: Forces prevent objects from passing through each other
2. **Friction Modeling**: Forces parallel to the contact surface resist sliding
3. **Restitution**: Energy conservation during impacts (bounciness)
4. **Contact Stabilization**: Numerical methods to maintain stable contacts

### Collision Pipeline

The collision detection process in Gazebo follows these stages:

1. **Broad Phase**: Quickly identify potentially colliding pairs using bounding volumes
2. **Narrow Phase**: Precisely compute contact points and forces
3. **Contact Resolution**: Apply forces to prevent penetration
4. **Constraint Solving**: Solve joint and contact constraints simultaneously

## Collision Algorithms

### Gazebo's Collision System

Gazebo uses ODE's collision detection system by default, with the following characteristics:

- **Primitive Shapes**: Box, sphere, cylinder, capsule
- **Mesh Collisions**: Convex hulls for complex geometries
- **Heightmap Collisions**: For terrain with elevation data

### Contact Modeling Approaches

#### Penalty Method
- Uses spring-damper systems to prevent penetration
- Computationally efficient but can cause vibrations
- Good for soft contacts and deformable objects

#### Constraint-Based Method
- Treats contacts as constraints to be satisfied
- More stable but computationally expensive
- Better for rigid body contacts

## Configuring Collision Properties

### Surface Parameters

Surface properties define how objects interact at contact points:

```xml
<collision name="foot_collision">
  <surface>
    <friction>
      <ode>
        <mu>0.8</mu>    <!-- Primary friction coefficient -->
        <mu2>0.8</mu2>  <!-- Secondary friction coefficient -->
        <fdir1>1 0 0</fdir1>  <!-- Friction direction for anisotropic friction -->
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.1</restitution_coefficient>  <!-- Bounciness -->
      <threshold>100000</threshold>  <!-- Velocity threshold for restitution -->
    </bounce>
    <contact>
      <ode>
        <soft_cfm>0</soft_cfm>      <!-- Constraint Force Mixing -->
        <soft_erp>0.2</soft_erp>    <!-- Error Reduction Parameter -->
        <kp>1e+13</kp>             <!-- Spring stiffness -->
        <kd>1</kd>                 <!-- Damping coefficient -->
        <max_vel>100.0</max_vel>    <!-- Maximum contact correction velocity -->
        <min_depth>0.001</min_depth> <!-- Penetration depth tolerance -->
      </ode>
    </contact>
  </surface>
</collision>
```

### Friction Coefficients for Humanoid Robots

For realistic humanoid robot simulation, consider these friction values:

- **Rubber on dry surface**: 0.8-1.0 (good for feet)
- **Leather on dry surface**: 0.5-0.8
- **Metal on metal**: 0.4-0.8
- **Wood on wood**: 0.25-0.5

### Restitution (Bounciness)

Restitution values for humanoid robots:
- **Human joints**: 0.05-0.2 (minimal bounce)
- **Rubber feet**: 0.1-0.3
- **Metallic surfaces**: 0.2-0.5

## Collision Detection Strategies

### Conservative vs Aggressive Detection

#### Conservative (Default)
- Lower false positive rate
- Better performance
- Slightly less accurate contact points

#### Aggressive Detection
- More accurate contact points
- Higher computational cost
- Better for precise applications

### Adaptive Collision Detection

For humanoid robots, consider using adaptive approaches:

```xml
<collision name="adaptive_collision">
  <surface>
    <contact>
      <ode>
        <soft_erp>0.1</soft_erp>    <!-- Lower ERP for stable contacts -->
        <soft_cfm>0.0001</soft_cfm> <!-- Higher CFM for stability -->
      </ode>
    </contact>
  </surface>
</collision>
```

## Handling Complex Collisions

### Multi-Contact Scenarios

Humanoid robots often experience multiple simultaneous contacts (e.g., walking):

```xml
<collision name="foot_collision">
  <geometry>
    <box size="0.2 0.1 0.05"/>  <!-- Approximate foot contact area -->
  </geometry>
  <surface>
    <contact>
      <ode>
        <max_contacts>10</max_contacts>  <!-- Allow multiple contact points -->
      </ode>
    </contact>
  </surface>
</collision>
```

### Contact Filtering

To prevent unwanted collisions (e.g., between adjacent links):

```xml
<collision name="collision">
  <surface>
    <contact>
      <collide_without_contact>0</collide_without_contact>
    </contact>
  </surface>
</collision>
```

## Collision Response Tuning

### Stability Considerations

For humanoid robot stability:

1. **Increase Contact Damping**: Higher `kd` values reduce oscillations
2. **Adjust ERP**: Lower values (0.1-0.2) for more stable contacts
3. **Balance CFM**: Higher values (0.0001-0.001) for stability
4. **Limit Max Velocity**: Prevent excessive contact corrections

### Performance vs Accuracy Trade-offs

| Setting | Performance Impact | Accuracy Impact | Best For |
|---------|-------------------|-----------------|----------|
| Low ERP | Lower stability | Higher accuracy | Precise simulation |
| High ERP | Higher stability | Lower accuracy | Stable simulation |
| High CFM | Higher stability | Lower accuracy | Stable simulation |
| Low CFM | Lower stability | Higher accuracy | Precise simulation |

## Validation and Testing

### Collision Validation Techniques

1. **Static Contact Test**: Place robot on flat surface, verify stability
2. **Dynamic Impact Test**: Drop robot from height, verify realistic response
3. **Sliding Test**: Push robot horizontally, verify friction behavior
4. **Multi-contact Test**: Create scenarios with multiple simultaneous contacts

### Debugging Collision Issues

Common collision problems and solutions:

#### Robot Penetrates Objects
- Increase `kp` (spring stiffness)
- Decrease `min_depth`
- Verify collision geometry alignment

#### Unstable Oscillations
- Increase `kd` (damping)
- Increase `soft_cfm`
- Decrease `soft_erp`

#### Excessive Sliding
- Increase `mu` (friction coefficient)
- Verify contact geometry alignment
- Check for proper mass distribution

## Advanced Collision Techniques

### Custom Contact Sensors

For monitoring specific contacts:

```xml
<gazebo reference="foot_link">
  <sensor name="foot_contact_sensor" type="contact">
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <contact>
      <collision>foot_collision</collision>
    </contact>
  </sensor>
</gazebo>
```

### Collision Groups and Masks

For complex collision filtering:

```xml
<collision name="collision">
  <surface>
    <contact>
      <collide_bitmask>1</collide_bitmask>  <!-- Collision group -->
    </contact>
  </surface>
</collision>
```

## Best Practices

1. **Conservative Start**: Begin with stable parameters, then tune for accuracy
2. **Consistent Units**: Ensure all parameters use consistent units (SI)
3. **Validation Testing**: Regularly test collision behavior with various scenarios
4. **Performance Monitoring**: Monitor simulation performance during tuning
5. **Real-world Validation**: Compare simulation behavior with real robot data when available

## Next Steps

After understanding collision detection and response, proceed to [Section 2.4: Parameter Tuning for Realistic Movement](../chapter-2/section-2-4-parameter-tuning) to learn how to fine-tune physics parameters for realistic humanoid robot movement.