# Chapter 5: Hands-on Exercise

## Exercise: URDF Modeling for Humanoid Robots

### Objective
Create and validate a complete URDF model for a humanoid robot, focusing on proper joint definitions and kinematic chains for bipedal movement.

### Part 1: URDF Validation and Analysis
1. First, let's validate the provided humanoid URDF model:
   ```bash
   check_urdf ~/path/to/your/humanoid.urdf
   ```

2. Generate a kinematic tree visualization:
   ```bash
   urdf_to_graphiz ~/path/to/your/humanoid.urdf
   ```

3. Examine the generated PDF files to understand the robot's structure.

### Part 2: Create a Simplified Humanoid URDF
Create a new file `simple_humanoid.urdf` with the following structure:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base Link -->
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
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
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
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.4" rpy="0 0 0"/>
  </joint>

  <!-- Left Hip -->
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
      <material name="blue"/>
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
    <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Yaw -->
    <limit lower="-0.5" upper="0.5" effort="200" velocity="1"/>
  </joint>

  <!-- Left Upper Leg -->
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
      <material name="blue"/>
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
    <axis xyz="0 1 0"/>  <!-- Pitch -->
    <limit lower="0" upper="1.57" effort="300" velocity="1"/>
  </joint>

  <!-- Left Lower Leg -->
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
      <material name="blue"/>
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
    <axis xyz="0 0 1"/>  <!-- Yaw -->
    <limit lower="-0.3" upper="0.3" effort="150" velocity="1"/>
  </joint>

  <!-- Left Foot -->
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
      <material name="blue"/>
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

  <!-- Right Leg (symmetric to left) -->
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
    <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
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
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="300" velocity="1"/>
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
    <axis xyz="0 0 1"/>
    <limit lower="-0.3" upper="0.3" effort="150" velocity="1"/>
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

### Part 3: URDF Analysis Exercise
Analyze the URDF model you created by answering these questions:

1. **Kinematic Chain Analysis**:
   - Trace the kinematic chain from `base_link` to `left_foot`
   - Identify the degrees of freedom for each joint
   - Calculate the total degrees of freedom in the left leg

2. **Joint Limitations**:
   - Why are the knee joint limits set to only allow positive angles?
   - What would happen if you set the ankle joint limits to ±1.57 radians?
   - How do the joint limits affect the robot's mobility?

3. **Inertial Properties**:
   - Why do the upper leg links have higher mass values than the lower leg links?
   - How do the inertia values relate to the mass and geometry of each link?
   - What would happen if all inertia values were set to zero?

### Part 4: URDF Validation and Testing
1. Validate your URDF file:
   ```bash
   check_urdf simple_humanoid.urdf
   ```

2. Generate the kinematic tree:
   ```bash
   urdf_to_graphiz simple_humanoid.urdf
   ```

3. Load the URDF in RViz for visualization:
   ```bash
   # Terminal 1
   ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat simple_humanoid.urdf)

   # Terminal 2
   ros2 run rviz2 rviz2
   # In RViz: Add RobotModel display and set Topic to your URDF
   ```

### Part 5: Xacro Conversion Exercise
Convert the URDF to Xacro format to demonstrate parametric modeling:

Create `simple_humanoid.xacro`:
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid_xacro">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931"/>
  <xacro:property name="torso_mass" value="10.0"/>
  <xacro:property name="leg_mass" value="5.0"/>
  <xacro:property name="hip_radius" value="0.06"/>
  <xacro:property name="leg_radius" value="0.05"/>
  <xacro:property name="leg_length" value="0.3"/>

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 8.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Macro for leg definition -->
  <xacro:macro name="leg" params="side prefix *origin">
    <joint name="${prefix}_hip_joint" type="revolute">
      <xacro:insert_block name="origin"/>
      <parent link="base_link"/>
      <child link="${prefix}_hip"/>
      <axis xyz="0 0 1"/>
      <limit lower="-0.5" upper="0.5" effort="200" velocity="1"/>
    </joint>

    <link name="${prefix}_hip">
      <inertial>
        <mass value="2.0"/>
        <origin xyz="0 0 -0.05" rpy="0 0 0"/>
        <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.002"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.05" rpy="0 0 0"/>
        <geometry>
          <cylinder length="0.1" radius="${hip_radius}"/>
        </geometry>
        <material name="${side == 'left' and 'blue' or 'green'}"/>
      </visual>
      <collision>
        <origin xyz="0 0 -0.05" rpy="0 0 0"/>
        <geometry>
          <cylinder length="0.1" radius="${hip_radius}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}_knee_joint" type="revolute">
      <parent link="${prefix}_hip"/>
      <child link="${prefix}_upper_leg"/>
      <origin xyz="0.0 0.0 -0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="1.57" effort="300" velocity="1"/>
    </joint>

    <link name="${prefix}_upper_leg">
      <inertial>
        <mass value="3.0"/>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.005"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${leg_length}" radius="${leg_radius}"/>
        </geometry>
        <material name="${side == 'left' and 'blue' or 'green'}"/>
      </visual>
      <collision>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${leg_length}" radius="${leg_radius}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}_ankle_joint" type="revolute">
      <parent link="${prefix}_upper_leg"/>
      <child link="${prefix}_lower_leg"/>
      <origin xyz="0.0 0.0 -${leg_length}" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-0.3" upper="0.3" effort="150" velocity="1"/>
    </joint>

    <link name="${prefix}_lower_leg">
      <inertial>
        <mass value="2.5"/>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.004"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${leg_length}" radius="${leg_radius * 0.9}"/>
        </geometry>
        <material name="${side == 'left' and 'blue' or 'green'}"/>
      </visual>
      <collision>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${leg_length}" radius="${leg_radius * 0.9}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}_foot_joint" type="fixed">
      <parent link="${prefix}_lower_leg"/>
      <child link="${prefix}_foot"/>
      <origin xyz="0.0 0.0 -${leg_length}" rpy="0 0 0"/>
    </joint>

    <link name="${prefix}_foot">
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
        <material name="${side == 'left' and 'blue' or 'green'}"/>
      </visual>
      <collision>
        <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="${torso_mass}"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
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
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.4" rpy="0 0 0"/>
  </joint>

  <!-- Use the macro to create both legs -->
  <xacro:leg side="left" prefix="left">
    <origin xyz="0.0 -0.05 0.0" rpy="0 0 0"/>
  </xacro:leg>

  <xacro:leg side="right" prefix="right">
    <origin xyz="0.0 0.05 0.0" rpy="0 0 0"/>
  </xacro:leg>

</robot>
```

### Part 6: Inverse Kinematics Simulation
Create a simple Python script to understand the relationship between joint angles and end-effector position:

```python
#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

def forward_kinematics(hip_angle, knee_angle, upper_leg_length=0.3, lower_leg_length=0.3):
    """
    Calculate foot position given hip and knee angles
    """
    # Calculate position of knee relative to hip
    knee_x = upper_leg_length * np.sin(hip_angle)
    knee_y = -upper_leg_length * np.cos(hip_angle)

    # Calculate position of foot relative to hip
    total_angle = hip_angle + knee_angle
    foot_x = knee_x + lower_leg_length * np.sin(total_angle)
    foot_y = knee_y - lower_leg_length * np.cos(total_angle)

    return foot_x, foot_y

def inverse_kinematics(foot_x, foot_y, upper_leg_length=0.3, lower_leg_length=0.3):
    """
    Calculate hip and knee angles to reach desired foot position
    """
    # Distance from hip to foot
    distance = np.sqrt(foot_x**2 + foot_y**2)

    # Check if position is reachable
    max_reach = upper_leg_length + lower_leg_length
    min_reach = abs(upper_leg_length - lower_leg_length)

    if distance > max_reach or distance < min_reach:
        print(f"Position ({foot_x:.2f}, {foot_y:.2f}) is not reachable!")
        return None, None

    # Calculate knee angle using law of cosines
    cos_knee_angle = (upper_leg_length**2 + lower_leg_length**2 - distance**2) / (2 * upper_leg_length * lower_leg_length)
    knee_angle = np.pi - np.arccos(np.clip(cos_knee_angle, -1, 1))

    # Calculate hip angle
    cos_hip_angle = (upper_leg_length**2 + distance**2 - lower_leg_length**2) / (2 * upper_leg_length * distance)
    alpha = np.arccos(np.clip(cos_hip_angle, -1, 1))
    beta = np.arctan2(foot_y, foot_x)
    hip_angle = beta + alpha

    return hip_angle, knee_angle

# Test the kinematics
print("Testing Forward and Inverse Kinematics:")

# Test 1: Forward kinematics
hip_angle = 0.2  # radians
knee_angle = 0.8  # radians
foot_x, foot_y = forward_kinematics(hip_angle, knee_angle)
print(f"Forward: Hip={hip_angle:.2f}, Knee={knee_angle:.2f} -> Foot=({foot_x:.2f}, {foot_y:.2f})")

# Test 2: Inverse kinematics
target_x, target_y = foot_x, foot_y  # Use result from forward kinematics
calc_hip, calc_knee = inverse_kinematics(target_x, target_y)
print(f"Inverse: Foot=({target_x:.2f}, {target_y:.2f}) -> Hip={calc_hip:.2f}, Knee={calc_knee:.2f}")

# Visualization
fig, ax = plt.subplots(1, 1, figsize=(10, 6))

# Draw leg at different poses
angles = np.linspace(0, np.pi/3, 5)
for i, angle in enumerate(angles):
    # Simple walking gait: hip moves forward, knee bends
    hip_ang = angle
    knee_ang = angle * 0.8

    foot_x, foot_y = forward_kinematics(hip_ang, knee_ang)

    # Draw the leg
    ax.plot([0, 0, foot_x], [0, -0.3, foot_y], 'o-', label=f'Pose {i+1}', linewidth=2)

ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.8, 0.2)
ax.set_aspect('equal')
ax.grid(True)
ax.legend()
ax.set_title('Humanoid Leg Poses')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')

plt.tight_layout()
plt.show()

print("\nKinematics Exercise Complete!")
print("This demonstrates how joint angles affect foot position in humanoid robots.")
```

### Discussion Questions
1. How does the URDF model affect the robot's ability to walk?
2. What are the trade-offs between detailed and simplified URDF models?
3. How would you extend this model to include arms and more complex joints?
4. What are the limitations of using fixed joint limits in dynamic environments?

### Extension Activities
1. Create a complete humanoid model with arms using the same principles
2. Implement a simple walking gait controller using the URDF model
3. Add sensors (IMU, cameras) to the URDF model
4. Export the URDF to a format compatible with physics simulators like Gazebo