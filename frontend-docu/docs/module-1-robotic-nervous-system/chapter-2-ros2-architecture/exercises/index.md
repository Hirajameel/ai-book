# Chapter 2: Hands-on Exercise

## Exercise: ROS 2 Environment Setup and Basic Node Creation

### Objective
Set up a ROS 2 environment and create your first ROS 2 nodes to understand the basic architecture.

### Prerequisites
- Ubuntu 22.04 with ROS 2 Humble installed
- Basic knowledge of terminal commands

### Part 1: Workspace Creation
1. Create a new ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. Build the empty workspace:
   ```bash
   colcon build --packages-select
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

### Part 2: Package Creation
Create a simple package for your exercises:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_pkg
```

### Part 3: Create a Publisher Node
1. Navigate to the package directory:
   ```bash
   cd ~/ros2_ws/src/my_robot_pkg/my_robot_pkg
   ```

2. Create a publisher node file `sensor_publisher.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class SensorPublisher(Node):
       def __init__(self):
           super().__init__('sensor_publisher')
           self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = f'Sensor reading: {self.i}'
           self.publisher_.publish(msg)
           self.get_logger().info(f'Publishing: "{msg.data}"')
           self.i += 1

   def main(args=None):
       rclpy.init(args=args)
       sensor_publisher = SensorPublisher()
       rclpy.spin(sensor_publisher)
       sensor_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Part 4: Create a Subscriber Node
1. Create a subscriber node file `motor_subscriber.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class MotorSubscriber(Node):
       def __init__(self):
           super().__init__('motor_subscriber')
           self.subscription = self.create_subscription(
               String,
               'motor_commands',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning

       def listener_callback(self, msg):
           self.get_logger().info(f'Received motor command: "{msg.data}"')

   def main(args=None):
       rclpy.init(args=args)
       motor_subscriber = MotorSubscriber()
       rclpy.spin(motor_subscriber)
       motor_subscriber.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Part 5: Update Package Configuration
1. Update `setup.py` in the package root to include your new nodes:
   ```python
   import os
   from glob import glob
   from setuptools import setup

   package_name = 'my_robot_pkg'

   setup(
       name=package_name,
       version='0.0.0',
       packages=[package_name],
       data_files=[
           ('share/ament_index/resource_index/packages',
               ['resource/' + package_name]),
           ('share/' + package_name, ['package.xml']),
       ],
       install_requires=['setuptools'],
       zip_safe=True,
       maintainer='your_name',
       maintainer_email='your_email@example.com',
       description='Simple robot package for learning ROS 2',
       license='Apache License 2.0',
       tests_require=['pytest'],
       entry_points={
           'console_scripts': [
               'sensor_publisher = my_robot_pkg.sensor_publisher:main',
               'motor_subscriber = my_robot_pkg.motor_subscriber:main',
           ],
       },
   )
   ```

### Part 6: Build and Run
1. Build your package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_pkg
   source install/setup.bash
   ```

2. Run the publisher in one terminal:
   ```bash
   ros2 run my_robot_pkg sensor_publisher
   ```

3. In another terminal, run the subscriber:
   ```bash
   ros2 run my_robot_pkg motor_subscriber
   ```

### Part 7: Understanding QoS Settings
Experiment with different QoS profiles to understand their impact:

1. Modify your publisher to use reliable QoS:
   ```python
   from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

   qos_profile = QoSProfile(
       reliability=ReliabilityPolicy.RELIABLE,
       history=HistoryPolicy.KEEP_LAST,
       depth=10
   )
   self.publisher_ = self.create_publisher(String, 'sensor_data', qos_profile)
   ```

### Discussion Questions
1. What happens when you run multiple instances of the publisher?
2. How does changing the QoS settings affect communication?
3. What would happen if you used a different topic name in publisher and subscriber?
4. How would you modify these nodes to work with real sensor data?

### Troubleshooting Tips
- Ensure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`
- Check that your workspace is properly sourced: `source install/setup.bash`
- Use `ros2 topic list` to verify topics are being published
- Use `ros2 node list` to verify nodes are running