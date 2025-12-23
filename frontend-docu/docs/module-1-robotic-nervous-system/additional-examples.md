# Additional Code Examples for ROS 2 Humanoid Robot

This document provides additional code examples that complement the chapters in Module 1.

## 1. Complete Humanoid Robot Controller Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
import numpy as np
import math

class HumanoidRobotController(Node):
    """
    Complete controller for a humanoid robot that integrates
    sensor processing, AI decision making, and motor control.
    """

    def __init__(self):
        super().__init__('humanoid_robot_controller')

        # Robot configuration
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        # Current and target joint positions
        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.target_positions = {name: 0.0 for name in self.joint_names}
        self.desired_velocities = {name: 0.0 for name in self.joint_names}

        # Robot state
        self.robot_state = 'idle'  # idle, walking, balancing, interacting
        self.balance_active = False
        self.walk_phase = 0.0

        # Publishers
        self.joint_cmd_publisher = self.create_publisher(
            JointState, 'joint_commands', 10)
        self.status_publisher = self.create_publisher(
            String, 'robot_status', 10)

        # Subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.imu_subscriber = self.create_subscription(
            Imu, 'imu_data', self.imu_callback, 10)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        # Balance control parameters
        self.balance_kp = 10.0  # Proportional gain
        self.balance_kd = 1.0   # Derivative gain

        self.get_logger().info('Humanoid Robot Controller initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions from joint state messages"""
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]

    def imu_callback(self, msg):
        """Process IMU data for balance control"""
        # Extract orientation (simplified - in real system use quaternions properly)
        roll = math.atan2(2.0 * (msg.orientation.w * msg.orientation.x +
                                msg.orientation.y * msg.orientation.z),
                         1.0 - 2.0 * (msg.orientation.x * msg.orientation.x +
                                     msg.orientation.y * msg.orientation.y))

        pitch = math.asin(2.0 * (msg.orientation.w * msg.orientation.y -
                                msg.orientation.z * msg.orientation.x))

        # Update balance control based on orientation
        if self.balance_active:
            self.adjust_balance(roll, pitch)

    def cmd_vel_callback(self, msg):
        """Handle velocity commands for locomotion"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if abs(linear_x) > 0.01 or abs(angular_z) > 0.01:
            self.robot_state = 'walking'
        else:
            self.robot_state = 'balancing'

    def adjust_balance(self, roll_error, pitch_error):
        """Apply balance correction to ankle joints"""
        # Simple PD controller for balance
        roll_correction = self.balance_kp * roll_error
        pitch_correction = self.balance_kp * pitch_error

        # Apply corrections to ankle joints
        self.target_positions['left_ankle_joint'] += roll_correction
        self.target_positions['right_ankle_joint'] += roll_correction
        self.target_positions['left_ankle_joint'] += pitch_correction * 0.5
        self.target_positions['right_ankle_joint'] -= pitch_correction * 0.5

        # Limit corrections to safe ranges
        for joint in ['left_ankle_joint', 'right_ankle_joint']:
            self.target_positions[joint] = max(-0.3, min(0.3, self.target_positions[joint]))

    def control_loop(self):
        """Main control loop that updates robot behavior"""
        # Update target positions based on current state
        self.update_target_positions()

        # Create and publish joint command
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = self.joint_names
        cmd_msg.position = [self.target_positions[name] for name in self.joint_names]
        cmd_msg.velocity = [self.desired_velocities[name] for name in self.joint_names]

        self.joint_cmd_publisher.publish(cmd_msg)

        # Publish status
        status_msg = String()
        status_msg.data = f"State: {self.robot_state}, Balance: {self.balance_active}"
        self.status_publisher.publish(status_msg)

    def update_target_positions(self):
        """Update target joint positions based on robot state"""
        if self.robot_state == 'walking':
            self.walk_pattern()
        elif self.robot_state == 'balancing':
            self.balance_pattern()
        elif self.robot_state == 'idle':
            self.idle_pattern()
        else:
            self.default_pattern()

    def walk_pattern(self):
        """Generate walking gait pattern"""
        self.walk_phase += 0.1

        # Simple walking gait - alternate leg movement
        left_leg_phase = self.walk_phase
        right_leg_phase = self.walk_phase + math.pi  # Opposite phase

        # Hip movement (raising and lowering legs)
        self.target_positions['left_hip_joint'] = 0.1 * math.sin(left_leg_phase)
        self.target_positions['right_hip_joint'] = 0.1 * math.sin(right_leg_phase)

        # Knee movement (bending to clear ground)
        self.target_positions['left_knee_joint'] = 0.5 + 0.3 * math.sin(left_leg_phase * 2)
        self.target_positions['right_knee_joint'] = 0.5 + 0.3 * math.sin(right_leg_phase * 2)

        # Ankle movement (maintaining foot orientation)
        self.target_positions['left_ankle_joint'] = -0.1 * math.sin(left_leg_phase)
        self.target_positions['right_ankle_joint'] = -0.1 * math.sin(right_leg_phase)

    def balance_pattern(self):
        """Return to neutral balance position"""
        for name in self.joint_names:
            # Smooth transition to neutral position
            current = self.current_positions[name]
            target = 0.0  # Neutral position
            # Apply a smooth transition
            self.target_positions[name] = current * 0.95 + target * 0.05

    def idle_pattern(self):
        """Standing position"""
        self.target_positions['left_hip_joint'] = 0.0
        self.target_positions['left_knee_joint'] = 0.0
        self.target_positions['left_ankle_joint'] = 0.0
        self.target_positions['right_hip_joint'] = 0.0
        self.target_positions['right_knee_joint'] = 0.0
        self.target_positions['right_ankle_joint'] = 0.0

    def default_pattern(self):
        """Default safe position"""
        self.idle_pattern()

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidRobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2. ROS 2 Service for Robot State Management

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from your_interfaces.srv import SetRobotState  # Custom service
from your_interfaces.action import WalkToPose  # Custom action
import time

class RobotStateManager(Node):
    """
    Service and action server for managing robot states
    and executing complex behaviors.
    """

    def __init__(self):
        super().__init__('robot_state_manager')

        # Service server for immediate state changes
        self.state_service = self.create_service(
            SetRobotState,
            'set_robot_state',
            self.set_robot_state_callback)

        # Action server for complex behaviors
        self.walk_action_server = ActionServer(
            self,
            WalkToPose,
            'walk_to_pose',
            execute_callback=self.execute_walk_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        self.current_state = 'idle'
        self.is_moving = False

        self.get_logger().info('Robot State Manager initialized')

    def set_robot_state_callback(self, request, response):
        """Handle immediate state change requests"""
        requested_state = request.state_command

        if requested_state in ['stand', 'sit', 'walk', 'stop', 'balance', 'idle']:
            old_state = self.current_state
            self.current_state = requested_state

            response.success = True
            response.message = f'State changed from {old_state} to {requested_state}'
            response.error_code = 0

            self.get_logger().info(f'State changed: {old_state} -> {requested_state}')
        else:
            response.success = False
            response.message = f'Invalid state command: {requested_state}'
            response.error_code = 1

        return response

    def goal_callback(self, goal_request):
        """Accept or reject walk goals"""
        self.get_logger().info(f'Received walk goal: {goal_request.target_pose}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject goal cancellation"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_walk_callback(self, goal_handle):
        """Execute the walk behavior"""
        self.get_logger().info('Starting walk execution')

        feedback_msg = WalkToPose.Feedback()
        result = WalkToPose.Result()

        target_pose = goal_handle.request.target_pose
        tolerance = goal_handle.request.tolerance

        # Simulate walking behavior
        steps = 0
        max_steps = 100  # Maximum steps to prevent infinite loops

        while steps < max_steps:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal canceled'
                return result

            # Simulate progress toward target
            # In a real system, this would use actual robot position feedback
            remaining_distance = max(0.0, 1.0 - (steps / max_steps))

            feedback_msg.current_pose = self.get_current_pose()  # Would need implementation
            feedback_msg.remaining_distance = remaining_distance
            feedback_msg.status = f'Walking... {remaining_distance:.2f}m remaining'

            goal_handle.publish_feedback(feedback_msg)

            # Check if we've reached the target
            if remaining_distance < tolerance:
                break

            # Simulate step delay
            time.sleep(0.1)
            steps += 1

        if steps < max_steps:
            goal_handle.succeed()
            result.success = True
            result.message = f'Reached target pose in {steps} steps'
        else:
            goal_handle.abort()
            result.success = False
            result.message = f'Failed to reach target after {max_steps} steps'

        self.get_logger().info(result.message)
        return result

    def get_current_pose(self):
        """Get current robot pose (placeholder)"""
        # This would interface with localization system in real implementation
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        return pose

def main(args=None):
    rclpy.init(args=args)
    state_manager = RobotStateManager()

    try:
        rclpy.spin(state_manager)
    except KeyboardInterrupt:
        pass
    finally:
        state_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. Sensor Fusion Node for Humanoid Robot

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import JointState, Imu, LaserScan
import statistics
import numpy as np
from collections import deque

class SensorFusionNode(Node):
    """
    Fuses data from multiple sensors to create a comprehensive
    understanding of the robot's state and environment.
    """

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Sensor data buffers
        self.imu_buffer = deque(maxlen=10)
        self.lidar_buffer = deque(maxlen=20)
        self.joint_buffer = deque(maxlen=5)

        # Processed data
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.balance_state = 'stable'
        self.obstacle_distance = float('inf')
        self.safety_status = 'safe'

        # Publishers
        self.fused_data_publisher = self.create_publisher(
            String, 'fused_sensor_data', 10)
        self.safety_publisher = self.create_publisher(
            Bool, 'safety_status', 10)

        # Subscribers
        self.imu_subscriber = self.create_subscription(
            Imu, 'imu_data', self.imu_callback, 10)
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)
        self.joint_subscriber = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)

        # Timer for fusion processing
        self.fusion_timer = self.create_timer(0.1, self.process_fusion)

        self.get_logger().info('Sensor Fusion Node initialized')

    def imu_callback(self, msg):
        """Process IMU data"""
        # Extract orientation (simplified)
        orientation = msg.orientation
        # Convert quaternion to roll/pitch/yaw would be done here
        # For simplicity, we'll just store the raw data
        self.imu_buffer.append({
            'orientation': (orientation.x, orientation.y, orientation.z, orientation.w),
            'angular_velocity': (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            'linear_acceleration': (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        })

        # Assess balance state from IMU data
        self.assess_balance_from_imu()

    def lidar_callback(self, msg):
        """Process LIDAR data"""
        # Store distance measurements
        self.lidar_buffer.append({
            'ranges': list(msg.ranges),
            'min_angle': msg.angle_min,
            'max_angle': msg.angle_max,
            'angle_increment': msg.angle_increment
        })

        # Find closest obstacle
        valid_ranges = [r for r in msg.ranges if 0.1 < r < 10.0]  # Filter invalid ranges
        if valid_ranges:
            self.obstacle_distance = min(valid_ranges)
        else:
            self.obstacle_distance = float('inf')

    def joint_callback(self, msg):
        """Process joint state data"""
        self.joint_buffer.append({
            'position': dict(zip(msg.name, msg.position)),
            'velocity': dict(zip(msg.name, msg.velocity)),
            'effort': dict(zip(msg.name, msg.effort))
        })

    def assess_balance_from_imu(self):
        """Determine balance state from IMU data"""
        if not self.imu_buffer:
            return

        # Get the most recent IMU data
        recent_imu = self.imu_buffer[-1]
        linear_acc = recent_imu['linear_acceleration']

        # Calculate tilt angles (simplified)
        tilt_x = np.arctan2(linear_acc[0], linear_acc[2])
        tilt_y = np.arctan2(linear_acc[1], linear_acc[2])

        # Determine balance state
        max_tilt = 0.3  # 0.3 radians ~ 17 degrees
        if abs(tilt_x) > max_tilt or abs(tilt_y) > max_tilt:
            self.balance_state = 'unstable'
        else:
            self.balance_state = 'stable'

    def process_fusion(self):
        """Process and fuse sensor data"""
        # Create fused data message
        fused_data = {
            'balance_state': self.balance_state,
            'obstacle_distance': self.obstacle_distance,
            'safety_status': self.safety_status,
            'timestamp': self.get_clock().now().nanoseconds
        }

        # Determine overall safety status
        self.safety_status = self.calculate_safety_status()

        # Publish fused data
        fused_msg = String()
        fused_msg.data = str(fused_data)
        self.fused_data_publisher.publish(fused_msg)

        # Publish safety status
        safety_msg = Bool()
        safety_msg.data = self.safety_status == 'safe'
        self.safety_publisher.publish(safety_msg)

        self.get_logger().info(f'Fused data: {fused_data}')

    def calculate_safety_status(self):
        """Calculate overall safety status from all sensors"""
        safety_factors = []

        # Balance safety
        if self.balance_state == 'unstable':
            safety_factors.append(False)
        else:
            safety_factors.append(True)

        # Obstacle safety (if too close)
        if self.obstacle_distance < 0.5:  # 50cm threshold
            safety_factors.append(False)
        else:
            safety_factors.append(True)

        # Return 'safe' if all factors are safe
        return 'safe' if all(safety_factors) else 'unsafe'

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. URDF Validation Script

```python
#!/usr/bin/env python3
"""
URDF Validation Script
This script validates URDF files and checks for common issues.
"""
import subprocess
import sys
import os
import xml.etree.ElementTree as ET

def validate_urdf_file(urdf_path):
    """Validate a URDF file using ROS 2 tools"""
    if not os.path.exists(urdf_path):
        print(f"Error: URDF file {urdf_path} does not exist")
        return False

    try:
        # Check URDF syntax
        print(f"Validating URDF file: {urdf_path}")
        result = subprocess.run(['check_urdf', urdf_path],
                              capture_output=True, text=True)
        if result.returncode != 0:
            print(f"URDF validation failed: {result.stderr}")
            return False
        else:
            print("✓ URDF syntax is valid")
            return True
    except FileNotFoundError:
        print("Error: check_urdf command not found. Is ROS 2 installed?")
        return False
    except Exception as e:
        print(f"Error validating URDF: {e}")
        return False

def analyze_urdf_structure(urdf_path):
    """Analyze URDF structure for humanoid-specific issues"""
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        # Check for robot name
        robot_name = root.get('name')
        print(f"Robot name: {robot_name}")

        # Count links and joints
        links = root.findall('.//link')
        joints = root.findall('.//joint')

        print(f"Number of links: {len(links)}")
        print(f"Number of joints: {len(joints)}")

        # Check for essential humanoid components
        link_names = [link.get('name') for link in links]
        joint_names = [joint.get('name') for joint in joints]

        essential_parts = {
            'torso': any('torso' in name.lower() or 'base' in name.lower() for name in link_names),
            'head': any('head' in name.lower() for name in link_names),
            'left_leg': any('left' in name.lower() and ('hip' in name.lower() or 'leg' in name.lower()) for name in joint_names),
            'right_leg': any('right' in name.lower() and ('hip' in name.lower() or 'leg' in name.lower()) for name in joint_names),
        }

        print("\nEssential humanoid parts check:")
        for part, present in essential_parts.items():
            status = "✓" if present else "✗"
            print(f"  {status} {part}: {'Present' if present else 'Missing'}")

        # Check joint types for legs
        leg_joints = [j for j in joints if 'leg' in j.get('name').lower() or
                     'hip' in j.get('name').lower() or
                     'knee' in j.get('name').lower() or
                     'ankle' in j.get('name').lower()]

        print(f"\nLeg joints found: {len(leg_joints)}")
        for joint in leg_joints:
            joint_type = joint.get('type')
            parent = joint.find('parent').get('link') if joint.find('parent') is not None else 'unknown'
            child = joint.find('child').get('link') if joint.find('child') is not None else 'unknown'
            print(f"  - {joint.get('name')} ({joint_type}): {parent} -> {child}")

        return True

    except ET.ParseError as e:
        print(f"Error parsing URDF XML: {e}")
        return False
    except Exception as e:
        print(f"Error analyzing URDF: {e}")
        return False

def generate_kinematic_tree(urdf_path):
    """Generate and display the kinematic tree"""
    try:
        result = subprocess.run(['urdf_to_graphiz', urdf_path],
                              capture_output=True, text=True)
        if result.returncode == 0:
            # Find the generated files
            base_name = os.path.splitext(urdf_path)[0]
            dot_file = f"{base_name}.gv"
            if os.path.exists(dot_file):
                print(f"✓ Kinematic tree generated: {dot_file}")
                # Read and display the tree structure
                with open(dot_file, 'r') as f:
                    content = f.read()
                    print("\nKinematic Tree Structure:")
                    print(content[:1000] + "..." if len(content) > 1000 else content)
            else:
                print("Could not find generated kinematic tree file")
        else:
            print(f"Error generating kinematic tree: {result.stderr}")
    except Exception as e:
        print(f"Error generating kinematic tree: {e}")

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 urdf_validator.py <urdf_file_path>")
        sys.exit(1)

    urdf_file = sys.argv[1]

    print("=== URDF Validation Tool ===\n")

    # Validate syntax
    if not validate_urdf_file(urdf_file):
        sys.exit(1)

    print()

    # Analyze structure
    analyze_urdf_structure(urdf_file)

    print()

    # Generate kinematic tree
    generate_kinematic_tree(urdf_file)

if __name__ == "__main__":
    main()
```

These additional code examples provide practical implementations that complement the theoretical content covered in the five chapters. They demonstrate real-world applications of the concepts discussed in the module.