# Chapter 3: Hands-on Exercise

## Exercise: Communication Patterns in ROS 2

### Objective
Implement and experiment with different ROS 2 communication patterns (Topics, Services, Actions) to understand their use cases in humanoid robotics.

### Part 1: Publisher/Subscriber with QoS Experimentation
1. Create a new package for this exercise:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python qos_experiment_pkg
   ```

2. Create a publisher node `qos_publisher.py` that publishes sensor data with different QoS profiles:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

   class QoSPublisher(Node):
       def __init__(self):
           super().__init__('qos_publisher')

           # Create publishers with different QoS profiles
           # Reliable communication (for critical data)
           reliable_qos = QoSProfile(
               reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST,
               depth=10
           )
           self.reliable_publisher = self.create_publisher(
               String, 'reliable_data', reliable_qos)

           # Best effort communication (for high-frequency data)
           best_effort_qos = QoSProfile(
               reliability=ReliabilityPolicy.BEST_EFFORT,
               history=HistoryPolicy.KEEP_LAST,
               depth=5
           )
           self.best_effort_publisher = self.create_publisher(
               String, 'best_effort_data', best_effort_qos)

           # Start timers for both publishers
           self.reliable_timer = self.create_timer(1.0, self.reliable_timer_callback)
           self.best_effort_timer = self.create_timer(0.1, self.best_effort_timer_callback)

           self.reliable_count = 0
           self.best_effort_count = 0

       def reliable_timer_callback(self):
           msg = String()
           msg.data = f'Reliable message #{self.reliable_count}'
           self.reliable_publisher.publish(msg)
           self.get_logger().info(f'Published: {msg.data}')
           self.reliable_count += 1

       def best_effort_timer_callback(self):
           msg = String()
           msg.data = f'Best effort message #{self.best_effort_count}'
           self.best_effort_publisher.publish(msg)
           self.best_effort_count += 1

   def main(args=None):
       rclpy.init(args=args)
       qos_publisher = QoSPublisher()
       rclpy.spin(qos_publisher)
       qos_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Part 2: Service Implementation
1. Create a service definition file `GetJointPosition.srv` in `ros2_ws/src/qos_experiment_pkg/qos_experiment_pkg/`:
   ```
   # Request
   string joint_name
   ---
   # Response
   float64 position
   bool success
   string message
   ```

2. Create a service server `joint_service_server.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from your_interfaces.srv import GetJointPosition  # You'll need to create this interface

   class JointServiceServer(Node):
       def __init__(self):
           super().__init__('joint_service_server')
           self.srv = self.create_service(
               GetJointPosition,
               'get_joint_position',
               self.get_joint_position_callback)

           # Simulate joint positions
           self.joint_positions = {
               'left_hip_joint': 0.1,
               'left_knee_joint': -0.5,
               'left_ankle_joint': 0.4,
               'right_hip_joint': -0.1,
               'right_knee_joint': 0.5,
               'right_ankle_joint': -0.4
           }

       def get_joint_position_callback(self, request, response):
           joint_name = request.joint_name
           if joint_name in self.joint_positions:
               response.position = self.joint_positions[joint_name]
               response.success = True
               response.message = f'Position for {joint_name}: {response.position}'
               self.get_logger().info(f'Service called for {joint_name}')
           else:
               response.position = 0.0
               response.success = False
               response.message = f'Joint {joint_name} not found'
               self.get_logger().warn(f'Invalid joint name: {joint_name}')

           return response

   def main(args=None):
       rclpy.init(args=args)
       service_server = JointServiceServer()
       rclpy.spin(service_server)
       service_server.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Part 3: Action Implementation
1. Create an action definition file `MoveJoint.action` in `ros2_ws/src/qos_experiment_pkg/qos_experiment_pkg/`:
   ```
   # Goal
   string joint_name
   float64 target_position
   float64 max_velocity
   ---
   # Result
   bool success
   string message
   float64 final_position
   ---
   # Feedback
   float64 current_position
   float64 remaining_distance
   string status
   ```

2. Create an action server `joint_action_server.py`:
   ```python
   import rclpy
   from rclpy.action import ActionServer, GoalResponse, CancelResponse
   from rclpy.node import Node
   from your_interfaces.action import MoveJoint  # You'll need to create this interface
   import time
   import threading

   class JointActionServer(Node):
       def __init__(self):
           super().__init__('joint_action_server')
           self._action_server = ActionServer(
               self,
               MoveJoint,
               'move_joint',
               execute_callback=self.execute_callback,
               goal_callback=self.goal_callback,
               cancel_callback=self.cancel_callback)

       def goal_callback(self, goal_request):
           self.get_logger().info('Received goal request')
           return GoalResponse.ACCEPT

       def cancel_callback(self, goal_handle):
           self.get_logger().info('Received cancel request')
           return CancelResponse.ACCEPT

       async def execute_callback(self, goal_handle):
           self.get_logger().info('Executing goal...')

           feedback_msg = MoveJoint.Feedback()
           result = MoveJoint.Result()

           joint_name = goal_handle.request.joint_name
           target_pos = goal_handle.request.target_position

           # Simulate joint movement
           current_pos = 0.0  # In real system, get current position
           step_size = 0.01

           while abs(current_pos - target_pos) > 0.01:
               if goal_handle.is_cancel_requested:
                   goal_handle.canceled()
                   result.success = False
                   result.message = 'Goal canceled'
                   return result

               # Move joint (simulated)
               if current_pos < target_pos:
                   current_pos += step_size
               else:
                   current_pos -= step_size

               # Update feedback
               feedback_msg.current_position = current_pos
               feedback_msg.remaining_distance = abs(current_pos - target_pos)
               feedback_msg.status = f'Moving to {target_pos:.2f}, current: {current_pos:.2f}'

               goal_handle.publish_feedback(feedback_msg)
               self.get_logger().info(f'Feedback: {feedback_msg.status}')

               time.sleep(0.1)  # Simulate movement time

           goal_handle.succeed()
           result.success = True
           result.message = f'Joint {joint_name} moved to {target_pos:.2f}'
           result.final_position = current_pos

           return result

   def main(args=None):
       rclpy.init(args=args)
       action_server = JointActionServer()
       rclpy.spin(action_server)
       action_server.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Part 4: Client Implementation
1. Create a client node `communication_client.py` that demonstrates all three patterns:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   from your_interfaces.srv import GetJointPosition
   from your_interfaces.action import MoveJoint
   from rclpy.action import ActionClient
   from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

   class CommunicationClient(Node):
       def __init__(self):
           super().__init__('communication_client')

           # Subscriber for sensor data
           self.subscription = self.create_subscription(
               String,
               'reliable_data',
               self.sensor_callback,
               QoSProfile(
                   reliability=ReliabilityPolicy.RELIABLE,
                   history=HistoryPolicy.KEEP_LAST,
                   depth=10
               )
           )

           # Service client
           self.cli = self.create_client(GetJointPosition, 'get_joint_position')
           while not self.cli.wait_for_service(timeout_sec=1.0):
               self.get_logger().info('Service not available, waiting again...')

           # Action client
           self._action_client = ActionClient(self, MoveJoint, 'move_joint')

           # Timer to trigger different communication patterns
           self.timer = self.create_timer(5.0, self.run_communication_demo)
           self.demo_step = 0

       def sensor_callback(self, msg):
           self.get_logger().info(f'Subscriber received: {msg.data}')

       def run_communication_demo(self):
           if self.demo_step == 0:
               self.call_service()
           elif self.demo_step == 1:
               self.send_action_goal()
           elif self.demo_step == 2:
               self.publish_message()

           self.demo_step = (self.demo_step + 1) % 3

       def call_service(self):
           request = GetJointPosition.Request()
           request.joint_name = 'left_knee_joint'

           future = self.cli.call_async(request)
           future.add_done_callback(self.service_callback)

       def service_callback(self, future):
           try:
               response = future.result()
               self.get_logger().info(f'Service response: {response.message}')
           except Exception as e:
               self.get_logger().error(f'Service call failed: {e}')

       def send_action_goal(self):
           goal_msg = MoveJoint.Goal()
           goal_msg.joint_name = 'left_knee_joint'
           goal_msg.target_position = 0.5
           goal_msg.max_velocity = 1.0

           self._action_client.wait_for_server()
           self._send_goal_future = self._action_client.send_goal_async(
               goal_msg,
               feedback_callback=self.feedback_callback)

           self._send_goal_future.add_done_callback(self.goal_response_callback)

       def goal_response_callback(self, future):
           goal_handle = future.result()
           if not goal_handle.accepted:
               self.get_logger().info('Goal rejected')
               return

           self.get_logger().info('Goal accepted')
           self._get_result_future = goal_handle.get_result_async()
           self._get_result_future.add_done_callback(self.result_callback)

       def feedback_callback(self, feedback_msg):
           self.get_logger().info(f'Received feedback: {feedback_msg.feedback.status}')

       def result_callback(self, future):
           result = future.result().result
           self.get_logger().info(f'Result: {result.message}')

       def publish_message(self):
           # This would typically be a publisher, but we're using existing subscriber
           # to demonstrate the concept
           self.get_logger().info('Demonstrating publish/subscribe pattern')

   def main(args=None):
       rclpy.init(args=args)
       client = CommunicationClient()
       rclpy.spin(client)
       client.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Part 5: Build and Test
1. Update `package.xml` to include message dependencies:
   ```xml
   <depend>std_msgs</depend>
   <depend>action_msgs</depend>
   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select qos_experiment_pkg
   source install/setup.bash
   ```

3. Run the different components in separate terminals:
   - Terminal 1: `ros2 run qos_experiment_pkg qos_publisher`
   - Terminal 2: `ros2 run qos_experiment_pkg joint_service_server`
   - Terminal 3: `ros2 run qos_experiment_pkg joint_action_server`
   - Terminal 4: `ros2 run qos_experiment_pkg communication_client`

### Discussion Questions
1. When would you use each communication pattern in a humanoid robot?
2. How do QoS settings affect real-time performance?
3. What are the advantages and disadvantages of each pattern?
4. How would you handle communication failures in each pattern?

### Performance Analysis
Monitor the performance of each communication pattern:
- Use `ros2 topic hz` to check message frequency
- Use `ros2 service list` to verify services are available
- Use `ros2 action list` to verify actions are available
- Observe the logs to understand timing differences