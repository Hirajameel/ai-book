---
title: Chapter 5 - Robot Description Format and Skeletal Modeling
sidebar_label: Chapter 5 Robot Description Format and Skeletal Modeling
---

# Chapter 5: Robot Description Format and Skeletal Modeling

## Introduction to URDF for Humanoid Robots

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. For humanoid robots, URDF defines the physical structure including links (rigid bodies), joints (connections between links), and their kinematic relationships. This chapter provides a comprehensive guide to creating detailed URDF models for humanoid robots.

## URDF Fundamentals

A URDF file describes a robot as a tree structure of links connected by joints. For humanoid robots, this typically includes:
- A base link (usually the torso)
- Limbs (arms and legs) with multiple joints
- End effectors (hands, feet)
- Sensors and other components

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <!-- Visual properties for display -->
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>

    <!-- Collision properties for physics simulation -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>

    <!-- Inertial properties for dynamics -->
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="torso_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.3" rpy="0 0 0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="light_grey"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>
</robot>
```

## Complete Humanoid URDF Structure

Here's a detailed URDF model for a humanoid robot with focus on hip, knee, and ankle joints as specified:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- MATERIALS -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- BASE LINK (TORSO) -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- HEAD -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.4" rpy="0 0 0"/>
  </joint>

  <!-- LEFT ARM -->
  <link name="left_shoulder">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.0005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder"/>
    <origin xyz="0.15 0.0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.0 0.0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.0008"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.035"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.035"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0.0 0.0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <!-- RIGHT ARM (symmetric to left) -->
  <link name="right_shoulder">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.0005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_shoulder"/>
    <origin xyz="-0.15 0.0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_upper_arm">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.0 0.0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.0008"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.035"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.035"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_wrist_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0.0 0.0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>

  <!-- LEFT LEG (HIP, KNEE, ANKLE) -->
  <link name="left_hip">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.002"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.06"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.06"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0.05 -0.05 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Yaw joint -->
    <limit lower="-0.5" upper="0.5" effort="200" velocity="1"/>
  </joint>

  <link name="left_upper_leg">
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_hip"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.0 0.0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Pitch joint -->
    <limit lower="0" upper="2.0" effort="300" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.004"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.045"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.045"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0.0 0.0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Yaw joint -->
    <limit lower="-0.5" upper="0.5" effort="150" velocity="1"/>
  </joint>

  <link name="left_foot">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.004"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_foot_joint" type="fixed">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0.0 0.0 -0.3" rpy="0 0 0"/>
  </joint>

  <!-- RIGHT LEG (symmetric to left) -->
  <link name="right_hip">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.002"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.06"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.06"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="0.05 0.05 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Yaw joint -->
    <limit lower="-0.5" upper="0.5" effort="200" velocity="1"/>
  </joint>

  <link name="right_upper_leg">
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_hip"/>
    <child link="right_upper_leg"/>
    <origin xyz="0.0 0.0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Pitch joint -->
    <limit lower="0" upper="2.0" effort="300" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.004"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.045"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.045"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0.0 0.0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Yaw joint -->
    <limit lower="-0.5" upper="0.5" effort="150" velocity="1"/>
  </joint>

  <link name="right_foot">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.004"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_foot_joint" type="fixed">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0.0 0.0 -0.3" rpy="0 0 0"/>
  </joint>

</robot>
```

## Humanoid-Specific Joint Considerations

### Hip Joints
Hip joints in humanoid robots typically require 3 degrees of freedom (DOF) to enable proper locomotion:
- **Yaw**: Rotation around vertical axis (turning)
- **Pitch**: Forward/backward movement
- **Roll**: Lateral tilting

For this simplified model, we've implemented yaw joints for basic turning capability, with the understanding that full 3-DOF hips would require additional joints.

### Knee Joints
Knee joints are primarily single-DOF, allowing flexion and extension:
- **Range**: Typically 0° to ~160° for human-like movement
- **Torque**: Requires high torque for supporting body weight during walking
- **Limitations**: Should prevent hyperextension

### Ankle Joints
Ankle joints provide crucial balance capabilities:
- **Pitch**: Dorsiflexion and plantarflexion
- **Roll**: Inversion and eversion
- **Stability**: Critical for bipedal locomotion

## Kinematic Modeling for Bipedal Movement

### Forward Kinematics
Forward kinematics calculates the position of the end effector (foot or hand) given joint angles. This is essential for:
- Planning movement trajectories
- Maintaining balance
- Avoiding collisions

### Inverse Kinematics
Inverse kinematics calculates required joint angles to achieve a desired end effector position. This is crucial for:
- Walking pattern generation
- Reaching movements
- Balance recovery

## URDF Validation

To validate your URDF model, use the following ROS 2 tools:

```bash
# Check URDF syntax
check_urdf /path/to/your/robot.urdf

# Visualize the kinematic tree
urdf_to_graphiz /path/to/your/robot.urdf

# Load in RViz for visualization
ros2 run rviz2 rviz2
# Then add RobotModel display and specify your URDF
```

### Validation Example Script

```python
#!/usr/bin/env python3
import subprocess
import sys
import os

def validate_urdf(urdf_path):
    """Validate URDF file using ROS 2 tools"""
    if not os.path.exists(urdf_path):
        print(f"Error: URDF file {urdf_path} does not exist")
        return False

    try:
        # Check URDF syntax
        result = subprocess.run(['check_urdf', urdf_path],
                              capture_output=True, text=True)
        if result.returncode != 0:
            print(f"URDF validation failed: {result.stderr}")
            return False
        else:
            print("URDF syntax is valid")
            return True
    except FileNotFoundError:
        print("Error: check_urdf command not found. Is ROS 2 installed?")
        return False
    except Exception as e:
        print(f"Error validating URDF: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 validate_urdf.py <urdf_file_path>")
        sys.exit(1)

    urdf_file = sys.argv[1]
    success = validate_urdf(urdf_file)
    sys.exit(0 if success else 1)
```

## Xacro for Complex URDFs

For more complex humanoid robots, Xacro (XML Macros) can simplify URDF creation:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_mass" value="10.0" />
  <xacro:property name="leg_mass" value="5.0" />

  <!-- Macro for leg definition -->
  <xacro:macro name="leg" params="side prefix">
    <link name="${prefix}_hip">
      <inertial>
        <mass value="2.0"/>
        <origin xyz="0 0 -0.05" rpy="0 0 0"/>
        <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.002"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.05" rpy="0 0 0"/>
        <geometry>
          <cylinder length="0.1" radius="0.06"/>
        </geometry>
        <material name="${side == 'left' and 'green' or 'red'}"/>
      </visual>
    </link>

    <joint name="${prefix}_hip_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_hip"/>
      <origin xyz="${side == 'left' and '0.05 -0.05 0.0' or '0.05 0.05 0.0'}" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-0.5" upper="0.5" effort="200" velocity="1"/>
    </joint>

    <!-- Additional joints and links for the leg -->
    <!-- ... -->
  </xacro:macro>

  <!-- Use the macro to create both legs -->
  <xacro:leg side="left" prefix="left"/>
  <xacro:leg side="right" prefix="right"/>

</robot>
```

## Performance Considerations

:::info
**Hardware Note**: When working with complex humanoid URDFs on Jetson Orin Nano:
- Simplify collision geometries (use boxes instead of complex meshes)
- Reduce the number of links if real-time performance is required
- Use convex hulls for collision detection instead of detailed meshes
- Consider using simplified URDF for real-time control and detailed URDF for simulation
:::

## Integration with ROS 2 Systems

URDF models integrate with ROS 2 through:

1. **Robot State Publisher**: Publishes TF transforms for the robot's kinematic structure
2. **Joint State Publisher**: Publishes joint positions for visualization
3. **Controllers**: Use URDF for inverse kinematics and motion planning
4. **Simulation**: Gazebo and other simulators use URDF for physics simulation

Example launch file to load and visualize the URDF:

```xml
<launch>
  <!-- Load URDF to parameter server -->
  <param name="robot_description"
         value="$(find-pkg-share your_robot_description)/urdf/humanoid.urdf"/>

  <!-- Start robot state publisher -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(var robot_description)"/>
  </node>

  <!-- Start joint state publisher (for visualization) -->
  <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" name="joint_state_publisher_gui"/>
</launch>
```

## Summary

This chapter provided a comprehensive overview of URDF for humanoid robots, including detailed examples of hip, knee, and ankle joints essential for bipedal movement. We covered the fundamental structure of URDF files, validation techniques, and advanced topics like Xacro for complex models. The URDF model serves as the bridge between the abstract AI decisions and the physical robot, enabling proper simulation and control of humanoid systems.

With this foundation in URDF modeling, your humanoid robot now has the complete physical representation needed to work with the AI systems and ROS 2 communication patterns covered in previous chapters.