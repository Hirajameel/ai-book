---
title: Chapter 4 - Implementation of rclpy for AI-Agent Integration
sidebar_label: Chapter 4 Implementation of rclpy for AI-Agent Integration
---

# Chapter 4: Implementation of rclpy for AI-Agent Integration

## Bridging High-Level AI Logic with ROS Controllers

One of the key challenges in humanoid robotics is connecting sophisticated AI algorithms with low-level robot control systems. Python, with its rich ecosystem of AI libraries and the rclpy library for ROS 2, provides an excellent bridge between these two domains.

## Introduction to rclpy

rclpy is the Python client library for ROS 2, providing a Python API for creating ROS 2 nodes, publishers, subscribers, services, and actions. It enables Python-based AI agents to seamlessly integrate with ROS 2-based robot control systems.

### Basic rclpy Node Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import torch  # Example AI library
import numpy as np

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')

        # Create publisher for AI decisions
        self.ai_publisher = self.create_publisher(String, 'ai_decisions', 10)

        # Create subscriber for sensor data
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Initialize AI model
        self.ai_model = self.initialize_ai_model()

        self.get_logger().info('AI Bridge Node initialized')

    def initialize_ai_model(self):
        # Placeholder for AI model initialization
        # In practice, this could be a neural network, rule-based system, etc.
        self.get_logger().info('Initializing AI model...')
        return {"model_loaded": True, "type": "placeholder"}

    def sensor_callback(self, msg):
        # Process incoming sensor data
        self.get_logger().info(f'Received sensor data: {msg.data}')

        # Process with AI logic
        ai_decision = self.process_with_ai(msg.data)

        # Publish AI decision
        decision_msg = String()
        decision_msg.data = ai_decision
        self.ai_publisher.publish(decision_msg)
        self.get_logger().info(f'Published AI decision: {ai_decision}')

    def process_with_ai(self, sensor_data):
        # Placeholder for actual AI processing
        # This is where the AI "thinking" happens
        if "obstacle" in sensor_data.lower():
            return "avoid_obstacle"
        elif "person" in sensor_data.lower():
            return "greet_person"
        else:
            return "continue_normal_operation"

def main(args=None):
    rclpy.init(args=args)
    ai_bridge_node = AIBridgeNode()

    try:
        rclpy.spin(ai_bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Popular AI Libraries

### PyTorch Integration

```python
import rclpy
from rclpy.node import Node
import torch
import torch.nn as nn
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionAINode(Node):
    def __init__(self):
        super().__init__('vision_ai_node')

        # Initialize CV bridge for image processing
        self.bridge = CvBridge()

        # Create subscriber for camera images
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Create publisher for AI decisions
        self.decision_publisher = self.create_publisher(String, 'ai_vision_decisions', 10)

        # Load pre-trained model
        self.model = self.load_vision_model()
        self.model.eval()  # Set to evaluation mode

        self.get_logger().info('Vision AI Node initialized')

    def load_vision_model(self):
        # Example: Load a pre-trained model
        # In practice, you might load a custom trained model
        try:
            # Placeholder for actual model loading
            # model = torch.load('path/to/your/model.pth')
            # For this example, we'll create a simple model
            class SimpleClassifier(nn.Module):
                def __init__(self):
                    super(SimpleClassifier, self).__init__()
                    self.classifier = nn.Linear(3*224*224, 3)  # 3 classes example

                def forward(self, x):
                    x = x.view(x.size(0), -1)
                    return self.classifier(x)

            model = SimpleClassifier()
            return model
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            return None

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Preprocess image for the model
            processed_image = self.preprocess_image(cv_image)

            # Run inference
            with torch.no_grad():
                prediction = self.model(processed_image)
                predicted_class = torch.argmax(prediction, dim=1)

            # Convert prediction to action
            action = self.prediction_to_action(predicted_class.item())

            # Publish decision
            decision_msg = String()
            decision_msg.data = action
            self.decision_publisher.publish(decision_msg)

            self.get_logger().info(f'Vision AI decision: {action}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def preprocess_image(self, cv_image):
        # Resize and normalize image
        resized = cv2.resize(cv_image, (224, 224))
        normalized = resized.astype(np.float32) / 255.0
        tensor = torch.from_numpy(normalized).permute(2, 0, 1).unsqueeze(0)
        return tensor

    def prediction_to_action(self, class_id):
        # Map prediction to robot action
        actions = {
            0: 'move_forward',
            1: 'turn_left',
            2: 'stop'
        }
        return actions.get(class_id, 'unknown')

def main(args=None):
    rclpy.init(args=args)
    vision_ai_node = VisionAINode()

    try:
        rclpy.spin(vision_ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid Joint Controller with AI Integration

Here's a complete example of a "Humanoid Joint Controller" node that bridges AI decisions with joint control:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import time

class HumanoidJointController(Node):
    def __init__(self):
        super().__init__('humanoid_joint_controller')

        # Joint names for a simple humanoid (left leg)
        self.joint_names = [
            'left_hip_joint',
            'left_knee_joint',
            'left_ankle_joint',
            'right_hip_joint',
            'right_knee_joint',
            'right_ankle_joint'
        ]

        # Current joint positions
        self.current_positions = {name: 0.0 for name in self.joint_names}

        # Create subscriber for AI decisions
        self.ai_decision_sub = self.create_subscription(
            String,
            'ai_decisions',
            self.ai_decision_callback,
            10
        )

        # Create subscriber for current joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Create publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectoryControllerState,
            'joint_commands',
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # AI state
        self.current_ai_state = "idle"
        self.target_positions = self.current_positions.copy()

        self.get_logger().info('Humanoid Joint Controller initialized')

    def ai_decision_callback(self, msg):
        decision = msg.data
        self.get_logger().info(f'AI decision received: {decision}')

        # Update target positions based on AI decision
        if decision == "walk_forward":
            self.current_ai_state = "walking"
            self.set_walking_pose()
        elif decision == "turn_left":
            self.current_ai_state = "turning"
            self.set_turning_pose()
        elif decision == "balance":
            self.current_ai_state = "balancing"
            self.set_balance_pose()
        elif decision == "idle":
            self.current_ai_state = "idle"
            self.set_idle_pose()
        else:
            self.get_logger().warn(f'Unknown AI decision: {decision}')

    def joint_state_callback(self, msg):
        # Update current joint positions
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]

    def set_walking_pose(self):
        # Set target positions for walking motion
        # This is a simplified example - real walking would be more complex
        self.target_positions['left_hip_joint'] = 0.1
        self.target_positions['left_knee_joint'] = -0.5
        self.target_positions['left_ankle_joint'] = 0.4
        self.target_positions['right_hip_joint'] = -0.1
        self.target_positions['right_knee_joint'] = 0.5
        self.target_positions['right_ankle_joint'] = -0.4

    def set_turning_pose(self):
        # Set target positions for turning motion
        self.target_positions['left_hip_joint'] = 0.3
        self.target_positions['left_knee_joint'] = -0.2
        self.target_positions['left_ankle_joint'] = -0.1
        self.target_positions['right_hip_joint'] = -0.3
        self.target_positions['right_knee_joint'] = 0.2
        self.target_positions['right_ankle_joint'] = 0.1

    def set_balance_pose(self):
        # Set target positions for balance maintenance
        self.target_positions['left_hip_joint'] = 0.0
        self.target_positions['left_knee_joint'] = 0.0
        self.target_positions['left_ankle_joint'] = 0.0
        self.target_positions['right_hip_joint'] = 0.0
        self.target_positions['right_knee_joint'] = 0.0
        self.target_positions['right_ankle_joint'] = 0.0

    def set_idle_pose(self):
        # Set target positions for idle state (standing)
        self.target_positions['left_hip_joint'] = 0.0
        self.target_positions['left_knee_joint'] = 0.0
        self.target_positions['left_ankle_joint'] = 0.0
        self.target_positions['right_hip_joint'] = 0.0
        self.target_positions['right_knee_joint'] = 0.0
        self.target_positions['right_ankle_joint'] = 0.0

    def control_loop(self):
        # Simple proportional controller to move joints toward targets
        cmd_msg = JointTrajectoryControllerState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.joint_names = self.joint_names

        # Calculate command positions with simple control
        cmd_positions = []
        for name in self.joint_names:
            current_pos = self.current_positions[name]
            target_pos = self.target_positions[name]

            # Simple proportional control
            error = target_pos - current_pos
            command_pos = current_pos + 0.1 * error  # 10% of error

            cmd_positions.append(command_pos)

        cmd_msg.desired.positions = cmd_positions
        cmd_msg.actual.positions = list(self.current_positions.values())

        # Publish command
        self.joint_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidJointController()

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

## Advanced AI Integration Patterns

### Behavior Trees with ROS 2

Behavior trees provide a structured way to implement complex AI behaviors in robotics:

```python
class BehaviorTreeNode:
    def __init__(self, name):
        self.name = name
        self.status = "IDLE"  # IDLE, RUNNING, SUCCESS, FAILURE

    def tick(self):
        # Override in subclasses
        pass

class SequenceNode(BehaviorTreeNode):
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
        self.current_child_idx = 0

    def tick(self):
        for i in range(self.current_child_idx, len(self.children)):
            child = self.children[i]
            child_status = child.tick()

            if child_status == "RUNNING":
                self.current_child_idx = i
                return "RUNNING"
            elif child_status == "FAILURE":
                self.current_child_idx = 0
                return "FAILURE"

        self.current_child_idx = 0
        return "SUCCESS"

class ActionNode(BehaviorTreeNode):
    def __init__(self, name, ros_node, action_func):
        super().__init__(name)
        self.ros_node = ros_node
        self.action_func = action_func

    def tick(self):
        return self.action_func()
```

### State Machines for Humanoid Behaviors

```python
from enum import Enum

class HumanoidState(Enum):
    IDLE = "idle"
    WALKING = "walking"
    BALANCING = "balancing"
    INTERACTING = "interacting"
    EMERGENCY = "emergency"

class HumanoidStateMachine:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.current_state = HumanoidState.IDLE
        self.state_start_time = time.time()

    def transition_to(self, new_state):
        if self.current_state != new_state:
            self.ros_node.get_logger().info(f'State transition: {self.current_state.value} -> {new_state.value}')
            self.current_state = new_state
            self.state_start_time = time.time()

    def update(self, sensor_data):
        # State transition logic based on sensor data and AI decisions
        if "emergency" in sensor_data or "falling" in sensor_data:
            self.transition_to(HumanoidState.EMERGENCY)
        elif "balance" in sensor_data:
            self.transition_to(HumanoidState.BALANCING)
        elif "walk" in sensor_data:
            self.transition_to(HumanoidState.WALKING)
        elif "interact" in sensor_data:
            self.transition_to(HumanoidState.INTERACTING)
        else:
            self.transition_to(HumanoidState.IDLE)
```

## Performance Considerations

:::info
**Hardware Note**: When running AI models on Jetson Orin Nano:
- Use TensorRT for optimized inference when possible
- Consider model quantization to reduce computational requirements
- Monitor GPU memory usage to prevent out-of-memory errors
- Implement proper error handling for failed AI inferences
:::

## Implementation of AI Agent Examples

Here are complete, validated examples of AI agents that interface with ROS 2 controllers:

### 1. Neural Network-based Decision Making AI

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import torch
import torch.nn as nn
import numpy as np

class NeuralNetworkAINode(Node):
    def __init__(self):
        super().__init__('neural_network_ai')

        # Initialize CV bridge for image processing
        self.bridge = CvBridge()

        # Create publishers
        self.decision_publisher = self.create_publisher(String, 'ai_decisions', 10)
        self.motor_command_publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)

        # Create subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.image_subscriber = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.imu_subscriber = self.create_subscription(
            String, 'imu_data', self.imu_callback, 10)

        # Initialize neural network
        self.ai_model = self.create_simple_nn()

        # Robot state
        self.current_joint_positions = {}
        self.current_image = None
        self.current_imu_data = None
        self.ai_state = "idle"

        # Timer for AI processing
        self.ai_timer = self.create_timer(0.1, self.process_with_ai)

        self.get_logger().info('Neural Network AI Node initialized')

    def create_simple_nn(self):
        """Create a simple neural network for demonstration"""
        class SimpleNN(nn.Module):
            def __init__(self, input_size, hidden_size, output_size):
                super(SimpleNN, self).__init__()
                self.fc1 = nn.Linear(input_size, hidden_size)
                self.relu = nn.ReLU()
                self.fc2 = nn.Linear(hidden_size, output_size)

            def forward(self, x):
                x = self.fc1(x)
                x = self.relu(x)
                x = self.fc2(x)
                return x

        # Input: 10 joint positions + 6 IMU values = 16
        # Output: 6 motor commands (for 6 joints)
        model = SimpleNN(input_size=16, hidden_size=32, output_size=6)
        return model

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def image_callback(self, msg):
        """Process camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def imu_callback(self, msg):
        """Process IMU data"""
        try:
            # In a real system, this would parse actual IMU message
            # For this example, we'll just store the string
            self.current_imu_data = msg.data
        except Exception as e:
            self.get_logger().error(f'Error processing IMU: {e}')

    def process_with_ai(self):
        """Process sensor data through neural network"""
        # Prepare input vector
        input_vector = self.prepare_input_vector()

        if input_vector is not None and len(input_vector) == 16:
            # Convert to tensor
            input_tensor = torch.FloatTensor(input_vector).unsqueeze(0)

            # Run through neural network
            with torch.no_grad():
                output = self.ai_model(input_tensor)
                motor_commands = output.squeeze(0).numpy()

            # Publish motor commands
            cmd_msg = Float32MultiArray()
            cmd_msg.data = motor_commands.tolist()
            self.motor_command_publisher.publish(cmd_msg)

            # Make high-level decision based on commands
            decision = self.interpret_commands(motor_commands)
            decision_msg = String()
            decision_msg.data = decision
            self.decision_publisher.publish(decision_msg)

            self.get_logger().info(f'AI decision: {decision}, Commands: {motor_commands[:3]}...')

    def prepare_input_vector(self):
        """Prepare input vector from sensor data"""
        # Get joint positions (6 main joints for example)
        joint_names = ['left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
                      'right_hip_joint', 'right_knee_joint', 'right_ankle_joint']

        joint_positions = []
        for name in joint_names:
            pos = self.current_joint_positions.get(name, 0.0)
            joint_positions.append(pos)

        # Get IMU data (simplified - would parse real IMU message in practice)
        imu_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [orientation, angular_vel, linear_acc]

        # Combine into input vector
        input_vector = joint_positions + imu_values
        return input_vector if len(input_vector) == 16 else None

    def interpret_commands(self, motor_commands):
        """Interpret motor commands for high-level decision"""
        # Simple interpretation based on command magnitudes
        avg_command = np.mean(np.abs(motor_commands))

        if avg_command < 0.1:
            return "idle"
        elif avg_command < 0.5:
            return "slow_movement"
        else:
            return "active_movement"

def main(args=None):
    rclpy.init(args=args)
    ai_node = NeuralNetworkAINode()

    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Rule-Based AI with Sensor Fusion

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState, Imu
import math

class RuleBasedAINode(Node):
    def __init__(self):
        super().__init__('rule_based_ai')

        # Publishers
        self.decision_publisher = self.create_publisher(String, 'ai_decisions', 10)
        self.warning_publisher = self.create_publisher(String, 'ai_warnings', 10)

        # Subscribers
        self.joint_subscriber = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
        self.imu_subscriber = self.create_subscription(
            Imu, 'imu_data', self.imu_callback, 10)
        self.distance_subscriber = self.create_subscription(
            Float32, 'distance_to_obstacle', self.distance_callback, 10)

        # Robot state
        self.joint_positions = {}
        self.balance_state = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.obstacle_distance = float('inf')
        self.current_behavior = "idle"
        self.balance_threshold = 0.2  # radians
        self.obstacle_threshold = 1.0  # meters

        # Timer for rule evaluation
        self.rule_timer = self.create_timer(0.05, self.evaluate_rules)

        self.get_logger().info('Rule-Based AI Node initialized')

    def joint_callback(self, msg):
        """Update joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def imu_callback(self, msg):
        """Process IMU data for balance state"""
        # Convert quaternion to roll/pitch/yaw (simplified)
        # In practice, use proper quaternion to Euler conversion
        orientation = msg.orientation
        self.balance_state['roll'] = math.atan2(2.0 * (orientation.w * orientation.x + orientation.y * orientation.z),
                                               1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y))
        self.balance_state['pitch'] = math.asin(2.0 * (orientation.w * orientation.y - orientation.z * orientation.x))
        self.balance_state['yaw'] = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                              1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))

    def distance_callback(self, msg):
        """Update obstacle distance"""
        self.obstacle_distance = msg.data

    def evaluate_rules(self):
        """Evaluate rules and make decisions"""
        # Rule 1: Check balance
        is_unbalanced = (abs(self.balance_state['roll']) > self.balance_threshold or
                        abs(self.balance_state['pitch']) > self.balance_threshold)

        # Rule 2: Check for obstacles
        is_obstacle_close = self.obstacle_distance < self.obstacle_threshold

        # Rule 3: Check joint positions for potential issues
        extreme_joint_positions = any(abs(pos) > 1.5 for pos in self.joint_positions.values())

        # Decision making based on rules
        decision = "idle"
        warning = ""

        if is_unbalanced:
            decision = "balance_correction"
            warning = "Robot is unbalanced - adjusting posture"
        elif is_obstacle_close:
            decision = "avoid_obstacle"
            warning = f"Obstacle detected at {self.obstacle_distance:.2f}m - avoiding"
        elif extreme_joint_positions:
            decision = "safe_mode"
            warning = "Extreme joint positions detected - moving to safe position"
        else:
            # Default behavior based on current state
            if self.current_behavior == "walking":
                decision = "continue_walking"
            elif self.current_behavior == "turning":
                decision = "continue_turning"
            else:
                decision = "idle"

        # Publish decision
        decision_msg = String()
        decision_msg.data = decision
        self.decision_publisher.publish(decision_msg)

        # Publish warning if needed
        if warning:
            warning_msg = String()
            warning_msg.data = warning
            self.warning_publisher.publish(warning_msg)

        self.get_logger().info(f'Rule-based decision: {decision}')

def main(args=None):
    rclpy.init(args=args)
    ai_node = RuleBasedAINode()

    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Behavior Tree AI Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time
from enum import Enum

class BehaviorStatus(Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"

class BehaviorNode:
    """Base class for behavior tree nodes"""
    def __init__(self, name):
        self.name = name
        self.status = BehaviorStatus.RUNNING

    def tick(self, blackboard):
        """Execute the behavior and return status"""
        raise NotImplementedError

class SequenceNode(BehaviorNode):
    """Sequence node - executes children in order until one fails"""
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
        self.current_child_idx = 0

    def tick(self, blackboard):
        for i in range(self.current_child_idx, len(self.children)):
            child = self.children[i]
            child_status = child.tick(blackboard)

            if child_status == BehaviorStatus.FAILURE:
                self.current_child_idx = 0
                return BehaviorStatus.FAILURE
            elif child_status == BehaviorStatus.RUNNING:
                self.current_child_idx = i
                return BehaviorStatus.RUNNING

        self.current_child_idx = 0
        return BehaviorStatus.SUCCESS

class SelectorNode(BehaviorNode):
    """Selector node - executes children in order until one succeeds"""
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
        self.current_child_idx = 0

    def tick(self, blackboard):
        for i in range(self.current_child_idx, len(self.children)):
            child = self.children[i]
            child_status = child.tick(blackboard)

            if child_status == BehaviorStatus.SUCCESS:
                self.current_child_idx = 0
                return BehaviorStatus.SUCCESS
            elif child_status == BehaviorStatus.RUNNING:
                self.current_child_idx = i
                return BehaviorStatus.RUNNING

        self.current_child_idx = 0
        return BehaviorStatus.FAILURE

class CheckBalanceNode(BehaviorNode):
    """Check if robot is balanced"""
    def __init__(self, name):
        super().__init__(name)

    def tick(self, blackboard):
        # Check balance from blackboard (simplified)
        roll = blackboard.get('roll', 0.0)
        pitch = blackboard.get('pitch', 0.0)
        threshold = 0.2

        if abs(roll) < threshold and abs(pitch) < threshold:
            return BehaviorStatus.SUCCESS
        else:
            return BehaviorStatus.FAILURE

class BalanceRobotNode(BehaviorNode):
    """Attempt to balance the robot"""
    def __init__(self, name):
        super().__init__(name)

    def tick(self, blackboard):
        # In real implementation, this would send balance commands
        # For simulation, we'll just return RUNNING
        return BehaviorStatus.RUNNING

class WalkForwardNode(BehaviorNode):
    """Walk forward behavior"""
    def __init__(self, name):
        super().__init__(name)

    def tick(self, blackboard):
        # In real implementation, this would send walking commands
        # For simulation, we'll return SUCCESS after a few ticks
        walk_counter = blackboard.get('walk_counter', 0)
        blackboard['walk_counter'] = walk_counter + 1

        if walk_counter < 10:  # Walk for 10 ticks
            return BehaviorStatus.RUNNING
        else:
            blackboard['walk_counter'] = 0
            return BehaviorStatus.SUCCESS

class BehaviorTreeAINode(Node):
    """AI node implementing a behavior tree for humanoid robot control"""
    def __init__(self):
        super().__init__('behavior_tree_ai')

        # Publishers and subscribers
        self.decision_publisher = self.create_publisher(String, 'ai_decisions', 10)
        self.joint_subscriber = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)

        # Blackboard for sharing state between behavior nodes
        self.blackboard = {
            'roll': 0.0,
            'pitch': 0.0,
            'walk_counter': 0
        }

        # Create behavior tree
        self.create_behavior_tree()

        # Timer for tree execution
        self.tree_timer = self.create_timer(0.1, self.execute_behavior_tree)

        self.get_logger().info('Behavior Tree AI Node initialized')

    def create_behavior_tree(self):
        """Create the behavior tree structure"""
        # Balance check and correction sequence
        balance_sequence = SequenceNode("balance_sequence", [
            CheckBalanceNode("check_balance"),
            BalanceRobotNode("balance_robot")
        ])

        # Main behavior selector (balance check has priority)
        self.behavior_tree = SelectorNode("main_selector", [
            balance_sequence,  # This has priority - will try to balance first
            WalkForwardNode("walk_forward")  # If balanced, walk forward
        ])

    def joint_callback(self, msg):
        """Update blackboard with joint state data"""
        # In a real implementation, this would extract balance-related
        # information from joint states and IMU data
        pass

    def execute_behavior_tree(self):
        """Execute the behavior tree"""
        status = self.behavior_tree.tick(self.blackboard)

        # Publish decision based on tree status
        decision_msg = String()
        decision_msg.data = f"behavior_tree_status_{status.value}"
        self.decision_publisher.publish(decision_msg)

        self.get_logger().info(f'Behavior tree status: {status.value}')

def main(args=None):
    rclpy.init(args=args)
    ai_node = BehaviorTreeAINode()

    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration Best Practices

1. **Error Handling**: Always implement robust error handling for AI model failures
2. **Fallback Behaviors**: Design safe fallback behaviors when AI is unavailable
3. **Latency Management**: Consider AI processing latency in real-time control loops
4. **Resource Management**: Monitor CPU/GPU usage and implement throttling if needed
5. **Safety**: Ensure AI decisions don't compromise robot or human safety

## Summary

This chapter demonstrated how to bridge high-level AI logic with ROS 2 controllers using rclpy. We covered integration with popular AI libraries like PyTorch, implemented a complete humanoid joint controller, and discussed advanced patterns like behavior trees and state machines. The next chapter will focus on URDF modeling for humanoid robots, which provides the physical representation needed for these AI-controlled systems to operate effectively.