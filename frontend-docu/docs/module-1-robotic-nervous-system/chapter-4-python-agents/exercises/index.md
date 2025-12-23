# Chapter 4: Hands-on Exercise

## Exercise: AI-ROS Integration with rclpy

### Objective
Create a simple AI agent that integrates with ROS 2 using rclpy, implementing basic decision-making and control capabilities for a humanoid robot.

### Part 1: Simple Decision-Making AI Node
Create a node that makes simple decisions based on sensor input:

1. Create a new package for this exercise:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python ai_exercise_pkg
   ```

2. Create the AI decision node `simple_ai_node.py` with the following structure:
   - Publisher for AI decisions (`ai_decisions` topic)
   - Subscriber for sensor data (`sensor_data` topic)
   - Timer-based decision making loop
   - Simple rule-based decision logic

### Part 2: Humanoid Joint Controller
Implement a joint controller that responds to AI decisions:

1. Create the controller node `humanoid_controller.py`:
   - Subscribes to `ai_decisions` topic
   - Publishes joint commands to `joint_commands` topic
   - Maintains current joint positions
   - Maps AI decisions to joint movements

### Part 3: Integration Test
1. Launch both nodes in separate terminals
2. Simulate sensor data to trigger AI decisions
3. Observe how the controller responds to AI decisions
4. Verify the integration works correctly

### Sample Solution Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleAINode(Node):
    def __init__(self):
        super().__init__('simple_ai_node')

        # Publisher for AI decisions
        self.decision_publisher = self.create_publisher(String, 'ai_decisions', 10)

        # Subscriber for sensor data
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Timer for AI processing
        self.ai_timer = self.create_timer(1.0, self.ai_processing_loop)

        # AI state
        self.current_sensor_data = ""
        self.ai_state = "idle"

        self.get_logger().info('Simple AI Node initialized')

    def sensor_callback(self, msg):
        self.current_sensor_data = msg.data
        self.get_logger().info(f'Received sensor data: {msg.data}')

    def ai_processing_loop(self):
        # Simple rule-based AI
        decision = self.make_decision()

        # Publish decision
        decision_msg = String()
        decision_msg.data = decision
        self.decision_publisher.publish(decision_msg)

        self.get_logger().info(f'AI Decision: {decision}')

    def make_decision(self):
        # Simple decision logic based on sensor data
        if "obstacle" in self.current_sensor_data.lower():
            self.ai_state = "avoiding"
            return "avoid_obstacle"
        elif "person" in self.current_sensor_data.lower():
            self.ai_state = "interacting"
            return "greet_person"
        else:
            self.ai_state = "exploring"
            return "continue_normal_operation"

def main(args=None):
    rclpy.init(args=args)
    ai_node = SimpleAINode()

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

### Discussion Questions
1. How does the AI make decisions based on sensor data?
2. What are the challenges of real-time AI integration with ROS 2?
3. How would you improve the learning algorithm for better humanoid control?
4. What safety measures would you implement in a real humanoid robot AI system?