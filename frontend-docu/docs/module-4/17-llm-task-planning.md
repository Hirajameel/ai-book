---
id: llm-task-planning
title: LLMs as Robotic Task Planners
sidebar_label: Chapter 17 LLM Task Planning
---

# LLMs as Robotic Task Planners

## Overview

This chapter focuses on using Large Language Models as cognitive bridges to translate natural language commands into ROS 2 action sequences. We'll explore prompt engineering techniques for generating ROS 2 Goal messages from natural language input.

## Development Environment Setup

### Prerequisites

Before implementing the LLM integration, ensure your system meets the following requirements:

#### Hardware Requirements
- **NVIDIA Jetson Orin** (Edge deployment) or **RTX GPU** (Simulation)
- At least 8GB RAM (16GB recommended for local LLM inference)
- 50GB free disk space for models and dependencies

#### Software Requirements
- ROS 2 Humble Hawksbill (or newer)
- Python 3.8 or higher
- CUDA 11.8+ (for GPU acceleration)

### Installation Steps

1. **Install Ollama for local LLM inference** (recommended for privacy and reliability):
```bash
curl -fsSL https://ollama.ai/install.sh | sh
```

2. **Pull required models**:
```bash
# For reasoning tasks
ollama pull llama3:70b

# Alternative smaller model for resource-constrained environments
ollama pull llama3:8b
```

3. **Install Python dependencies**:
```bash
pip install openai  # For both OpenAI API and Ollama compatibility
pip install pydantic
pip install ros2
```

4. **Verify LLM access**:
```bash
# Test Ollama
ollama run llama3:8b
```

### Alternative: OpenAI API Setup

If using OpenAI API instead of local inference:

1. **Get API key** from [platform.openai.com](https://platform.openai.com)
2. **Install dependencies**:
```bash
pip install openai
```
3. **Set environment variable**:
```bash
export OPENAI_API_KEY='your-api-key-here'
```

## Reasoning-before-Acting Logic

The core concept of "reasoning-before-acting" involves having the LLM think through a command before generating action sequences:

```
User Command: "Go to the kitchen and bring me a cup"
                    ↓
        [LLM Reasoning Phase]
    - Identify locations (kitchen)
    - Identify objects (cup)
    - Plan navigation sequence
    - Plan manipulation sequence
                    ↓
        [Action Sequence Generation]
    1. Navigate to kitchen
    2. Locate cup
    3. Approach cup
    4. Grasp cup
    5. Return to user
```

## Python Parser for LLM JSON Output

Here's a Python parser that converts LLM JSON output into ROS 2 goals:

```python
import json
from typing import Dict, List, Any
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import rclpy

class LLMResponseParser:
    def __init__(self, node: Node):
        self.node = node
        self.action_publisher = node.create_publisher(String, 'llm_actions', 10)

    def parse_llm_response(self, llm_output: str) -> Dict[str, Any]:
        """
        Parse LLM JSON response and convert to ROS 2 action sequence
        Expected LLM output format:
        {
          "thoughts": "reasoning process...",
          "actions": [
            {
              "type": "navigation",
              "goal": {
                "x": 1.0,
                "y": 2.0,
                "theta": 0.0
              },
              "description": "Go to kitchen"
            },
            {
              "type": "object_detection",
              "target": "cup",
              "description": "Find the cup"
            }
          ]
        }
        """
        try:
            # Parse the JSON response from LLM
            response_data = json.loads(llm_output)

            # Validate the structure
            if 'actions' not in response_data:
                raise ValueError("LLM response missing 'actions' field")

            # Convert to ROS 2 action sequence
            action_sequence = self._convert_to_ros_actions(response_data['actions'])

            return action_sequence

        except json.JSONDecodeError as e:
            self.node.get_logger().error(f"Failed to parse LLM JSON response: {e}")
            return {"error": f"JSON parsing error: {str(e)}"}
        except Exception as e:
            self.node.get_logger().error(f"Error processing LLM response: {e}")
            return {"error": f"Processing error: {str(e)}"}

    def _convert_to_ros_actions(self, actions: List[Dict]) -> Dict[str, Any]:
        """Convert LLM action list to ROS 2 action sequence"""
        ros_actions = []

        for action in actions:
            action_type = action.get('type', 'unknown')

            if action_type == 'navigation':
                ros_action = self._create_navigation_action(action)
            elif action_type == 'object_detection':
                ros_action = self._create_object_detection_action(action)
            elif action_type == 'manipulation':
                ros_action = self._create_manipulation_action(action)
            else:
                ros_action = self._create_generic_action(action)

            ros_actions.append(ros_action)

        return {"actions": ros_actions, "count": len(ros_actions)}

    def _create_navigation_action(self, action: Dict) -> Dict:
        """Create a navigation action for ROS 2"""
        goal = action.get('goal', {})
        x = goal.get('x', 0.0)
        y = goal.get('y', 0.0)
        theta = goal.get('theta', 0.0)

        # Create a PoseStamped goal for Nav2
        nav_goal = {
            "action_type": "navigation",
            "goal_pose": {
                "position": {"x": x, "y": y, "z": 0.0},
                "orientation": self._euler_to_quaternion(0, 0, theta)
            },
            "description": action.get('description', 'Navigate to goal'),
            "timeout": action.get('timeout', 60.0)  # Default 60 seconds
        }

        return nav_goal

    def _create_object_detection_action(self, action: Dict) -> Dict:
        """Create an object detection action for ROS 2"""
        return {
            "action_type": "object_detection",
            "target_object": action.get('target', 'unknown'),
            "description": action.get('description', 'Detect object'),
            "confidence_threshold": action.get('confidence_threshold', 0.7)
        }

    def _create_manipulation_action(self, action: Dict) -> Dict:
        """Create a manipulation action for ROS 2"""
        return {
            "action_type": "manipulation",
            "action": action.get('manipulation_type', 'grasp'),
            "target": action.get('target', 'object'),
            "description": action.get('description', 'Manipulate object'),
            "gripper_position": action.get('gripper_position', 0.5)  # 0.0 to 1.0
        }

    def _create_generic_action(self, action: Dict) -> Dict:
        """Create a generic action for ROS 2"""
        return {
            "action_type": "generic",
            "command": action.get('command', 'unknown'),
            "parameters": action.get('parameters', {}),
            "description": action.get('description', 'Generic action')
        }

    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Dict:
        """Convert Euler angles to quaternion"""
        import math

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return {"x": x, "y": y, "z": z, "w": w}

    def publish_action_sequence(self, action_sequence: Dict):
        """Publish the action sequence to ROS 2 topic"""
        msg = String()
        msg.data = json.dumps(action_sequence)
        self.action_publisher.publish(msg)
        self.node.get_logger().info(f"Published action sequence with {action_sequence.get('count', 0)} actions")

# Example usage in a ROS 2 node
class LLMTaskPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_task_planner_node')
        self.parser = LLMResponseParser(self)

        # Create subscriber for LLM responses
        self.llm_subscriber = self.create_subscription(
            String,
            'llm_responses',
            self.llm_response_callback,
            10
        )

        self.get_logger().info("LLM Task Planner Node initialized")

    def llm_response_callback(self, msg):
        """Process incoming LLM responses"""
        self.get_logger().info(f"Received LLM response: {msg.data[:100]}...")

        # Parse the response
        action_sequence = self.parser.parse_llm_response(msg.data)

        if "error" not in action_sequence:
            # Publish the action sequence
            self.parser.publish_action_sequence(action_sequence)
        else:
            self.get_logger().error(f"Error processing LLM response: {action_sequence['error']}")

def main(args=None):
    rclpy.init(args=args)
    node = LLMTaskPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## System Prompt Template

Here's a "System Prompt" template for the LLM to act as a reliable robot controller:

```
You are an expert robotic task planner. Your role is to convert natural language commands into structured action sequences for a humanoid robot.

Follow these rules:
1. Always think step-by-step before generating actions
2. Consider safety and feasibility of each action
3. Break complex tasks into simple, executable steps
4. Use only the action types: navigation, object_detection, manipulation, generic
5. Include necessary parameters for each action
6. If information is missing, ask for clarification rather than guessing

Respond in valid JSON format with this structure:
{
  "thoughts": "your reasoning process",
  "actions": [
    {
      "type": "action_type",
      "parameters": {...},
      "description": "what this action does"
    }
  ]
}

Example: For "Go to the kitchen and bring me a red cup":
{
  "thoughts": "User wants a red cup from the kitchen. First navigate to kitchen, then find red cup, then grasp it, then return.",
  "actions": [
    {
      "type": "navigation",
      "goal": {"x": 5.0, "y": 3.0, "theta": 0.0},
      "description": "Navigate to kitchen area"
    },
    {
      "type": "object_detection",
      "target": "red cup",
      "description": "Locate the red cup in the environment"
    }
  ]
}
```

## Integration with ROS 2 Action Servers

The generated action sequences need to be processed by ROS 2 action servers:

1. **Navigation Action Server**: For `MapsToPose` goals
2. **Object Detection Action Server**: For `FindObject` goals
3. **Manipulation Action Server**: For `PickObject` goals

These will be implemented in the following chapters as part of the complete VLA system.

## Next Steps

In the next sections, we'll implement the complete action execution pipeline that takes these parsed action sequences and executes them on the robot hardware.