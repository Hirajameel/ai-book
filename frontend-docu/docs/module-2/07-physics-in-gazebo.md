---
title: Chapter 7 - Physics in Gazebo
sidebar_label: Chapter 7 Physics in Gazebo
id: 07-physics-in-gazebo
---

# Chapter 7: Physics in Gazebo for Humanoid Robots

## Understanding Gazebo's Physics Engine

Gazebo serves as the "Physics Truth" in the digital twin paradigm, using the Open Dynamics Engine (ODE) as its primary physics engine to provide realistic simulation of rigid body dynamics. For humanoid robots, understanding how to configure physical properties in Gazebo is crucial for achieving stable and realistic behavior that can be validated against real-world physics.

Gazebo handles the core physics calculations including gravity, collisions, and dynamics that determine the "truth" of how the robot behaves. This contrasts with Unity, which handles the "Visual Experience" by rendering what the robot sees without performing physics calculations.

## Configuring Gravity for Humanoid Stability

Gravity is the fundamental force that affects all humanoid robots. In Gazebo, you can configure gravity in the world file:

```xml
<sdf version="1.6">
  <world name="default">
    <gravity>0 0 -9.8</gravity>
    <!-- Standard Earth gravity -->
  </world>
</sdf>
```

For humanoid robots, realistic gravity values are crucial for:
- Testing balance control algorithms under real-world conditions
- Validating center of mass calculations during locomotion
- Ensuring proper weight distribution during bipedal walking
- Accurately simulating the effects of external forces on stability

Small variations in gravity (e.g., ±0.1 m/s²) can be used to test controller robustness.

## Inertia Configuration for Bipedal Robots

Proper inertia values are critical for humanoid stability and must accurately reflect the physical robot's mass distribution. Each link in the robot must have precise mass and inertia tensor values:

```xml
<link name="torso">
  <inertial>
    <mass>10.0</mass>
    <inertia>
      <ixx>0.1</ixx>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>0.1</iyy>
      <iyz>0.0</iyz>
      <izz>0.1</izz>
    </inertia>
  </inertial>
</link>
```

### Calculating Inertia for Humanoid Stability

For humanoid robots, the inertia tensor directly affects how the robot responds to forces and torques during movement:

- **Mass distribution**: The center of mass position significantly impacts balance stability
- **Moment of inertia**: The diagonal elements (ixx, iyy, izz) determine how easily the link rotates around each axis
- **Product of inertia**: Off-diagonal elements (ixy, ixz, iyz) should typically be zero for symmetric objects

### Guidelines for Humanoid Link Inertia

For stable humanoid locomotion, consider these guidelines when defining inertia:

1. **Torso/Body links**: Higher moments of inertia around lateral axes (ixx, iyy) for stability
2. **Limb links**: Lower moments of inertia to allow for faster movement
3. **Foot links**: Appropriate mass and inertia to provide stable ground contact

### Realistic Inertia Calculation Methods

To calculate accurate inertia values for humanoid robots:

1. **CAD-based calculation**: Use CAD software to compute exact inertia tensors from 3D models
2. **Approximation with primitives**: Model complex links as combinations of simple geometric shapes
3. **Empirical tuning**: Adjust values based on real robot behavior to match simulation

Example of a more realistic torso configuration:

```xml
<link name="torso">
  <inertial>
    <mass>15.0</mass>
    <pose>0.0 0.0 0.2 0 0 0</pose>  <!-- Center of mass offset -->
    <inertia>
      <ixx>0.3</ixx>     <!-- Higher around roll axis -->
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>0.4</iyy>     <!-- Higher around pitch axis -->
      <iyz>0.0</iyz>
      <izz>0.2</izz>     <!-- Lower around yaw axis -->
    </inertia>
  </inertial>
</link>
```

Properly configured inertia values are essential for realistic humanoid dynamics and stable balance control.

## Cross-Reference to Module 1

For a comprehensive understanding of how URDF models integrate with physics simulation, refer to Module 1's [Humanoid Kinematics in URDF](../module-1-robotic-nervous-system/humanoid-kinematics-urdf.md) chapter. The mass and inertia properties defined in your URDF directly affect the physics simulation in Gazebo, making the connection between kinematic modeling and dynamic simulation critical for humanoid robot development.

## Surface Friction for Bipedal Locomotion

Friction parameters are critical for humanoid stability and prevent the robot from slipping or sliding during walking and balance tasks. The contact friction between the robot's feet and the ground must be carefully tuned to match real-world conditions:

```xml
<collision name="foot_collision">
  <surface>
    <friction>
      <ode>
        <mu>0.8</mu>  <!-- Static friction coefficient -->
        <mu2>0.8</mu2> <!-- Dynamic friction coefficient -->
        <slip1>0.0</slip1>
        <slip2>0.0</slip2>
      </ode>
    </friction>
  </surface>
</collision>
```

For humanoid robots, friction coefficients between 0.7-1.0 typically provide stable walking on most surfaces. Values too low will cause the robot to slip, while values too high can cause unrealistic sliding behavior during dynamic movements.

### Contact Friction Tuning for Humanoid Stability

The friction parameters directly affect the robot's ability to maintain balance:

- **Static friction (mu)**: Determines the maximum force that can be applied before the foot starts to slip
- **Dynamic friction (mu2)**: Controls the friction force once slipping has begun
- **Slip parameters**: Help prevent numerical instabilities in the solver

For humanoid stability, it's recommended to set both `mu` and `mu2` to the same value to avoid discontinuous behavior during transitions between static and dynamic friction.

### Advanced Contact Parameters

For enhanced humanoid stability, consider these additional contact parameters:

```xml
<collision name="foot_collision">
  <surface>
    <contact>
      <ode>
        <soft_cfm>0.0001</soft_cfm>  <!-- Constraint Force Mixing -->
        <soft_erp>0.9</soft_erp>      <!-- Error Reduction Parameter -->
        <kp>1000000.0</kp>            <!-- Contact stiffness -->
        <kd>100.0</kd>                <!-- Contact damping -->
      </ode>
    </contact>
    <friction>
      <ode>
        <mu>0.8</mu>
        <mu2>0.8</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

These parameters help maintain stable contact between the robot's feet and the ground, which is essential for bipedal locomotion.

## ODE Parameters for Humanoid Stability

ODE provides several parameters that affect humanoid stability:

- **ERP (Error Reduction Parameter)**: Controls how quickly position errors are corrected
- **CFM (Constraint Force Mixing)**: Adds a small compliance to constraints
- **Max Vel**: Maximum velocity for contact joints

For humanoid robots, these parameters need careful tuning to achieve stable balance without excessive oscillation.

## Collision Detection in Gazebo

Collision detection is essential for humanoid robots to interact properly with the environment. Gazebo provides multiple collision detection algorithms:

- **Bullet**: Good for complex geometries
- **ODE**: Fast and stable for simple geometries
- **Simbody**: Accurate for complex systems

For humanoid robots, ODE is typically the best choice due to its balance of speed and stability.

### Collision Shapes for Humanoid Stability

The choice of collision shapes significantly impacts humanoid stability:

- **Primitive shapes** (boxes, spheres, cylinders): Fast computation, good for basic contact
- **Mesh shapes**: Accurate representation of complex geometries but computationally expensive
- **Compound shapes**: Combination of primitives for complex but efficient collision detection

For humanoid feet, using a simplified box collision instead of a complex mesh can improve stability while maintaining performance.

### Contact Stabilization

Proper contact stabilization is critical for humanoid balance:

- **Contact stiffness**: Controls how rigidly objects resist penetration
- **Damping**: Reduces oscillations during contact
- **Contact surface parameters**: Affect friction and slip behavior

Tuning these parameters helps prevent jittery behavior during bipedal locomotion.

## Best Practices for Physics Configuration

1. **Start with realistic values**: Use actual robot specifications for mass and dimensions
2. **Tune incrementally**: Adjust one parameter at a time to observe effects
3. **Validate with real robot**: Compare simulation behavior with physical robot when possible
4. **Consider environmental factors**: Adjust parameters for different surfaces and conditions

## Troubleshooting Common Physics Issues

- **Robot falls over**: Check center of mass, inertia values, and friction coefficients
- **Jittery movement**: Adjust ERP and CFM values
- **Penetration**: Increase collision margin or adjust solver parameters

---

Next Chapter: [Chapter 8 - High-Fidelity Rendering with Unity](./08-high-fidelity-rendering-unity.md)