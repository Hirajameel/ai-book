# Quickstart Guide: ROS 2 Humanoid Robot Module

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Python 3.10 or higher
- Basic understanding of robotics concepts

## Installation & Setup

### 1. Install ROS 2 Humble

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

### 2. Source ROS 2 Environment

```bash
source /opt/ros/humble/setup.bash
```

### 3. Create a Workspace

```bash
mkdir -p ~/ros2_humanoid_ws/src
cd ~/ros2_humanoid_ws
```

## Basic ROS 2 Node Example

### Create a Simple Publisher Node

Create `~/ros2_humanoid_ws/src/my_robot_pkg/my_robot_pkg/sensor_publisher.py`:

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

### Create a Simple Subscriber Node

Create `~/ros2_humanoid_ws/src/my_robot_pkg/my_robot_pkg/motor_subscriber.py`:

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

### Build and Run

```bash
cd ~/ros2_humanoid_ws
colcon build --packages-select my_robot_pkg
source install/setup.bash

# Terminal 1: Run the publisher
ros2 run my_robot_pkg sensor_publisher

# Terminal 2: Run the subscriber
ros2 run my_robot_pkg motor_subscriber
```

## Simple URDF Example

Create `~/ros2_humanoid_ws/src/my_robot_pkg/my_robot_pkg/urdf/simple_humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Hip Joint and Thigh Link -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="0 -0.05 -0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Knee Joint and Shin Link -->
  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

### Validate the URDF

```bash
# Check URDF syntax
check_urdf ~/ros2_humanoid_ws/src/my_robot_pkg/my_robot_pkg/urdf/simple_humanoid.urdf

# Visualize the robot
urdf_to_graphiz ~/ros2_humanoid_ws/src/my_robot_pkg/my_robot_pkg/urdf/simple_humanoid.urdf
```

## Python AI Agent Integration

Create `~/ros2_humanoid_ws/src/my_robot_pkg/my_robot_pkg/ai_agent.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AIAgent(Node):
    def __init__(self):
        super().__init__('ai_agent')

        # Subscribe to sensor data
        self.sensor_subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10)

        # Publish motor commands
        self.motor_publisher = self.create_publisher(
            String,
            'motor_commands',
            10)

        self.get_logger().info('AI Agent initialized')

    def sensor_callback(self, msg):
        # Process sensor data with AI logic
        sensor_data = msg.data
        self.get_logger().info(f'AI Agent received: {sensor_data}')

        # Generate motor command based on AI logic
        motor_command = self.process_with_ai(sensor_data)

        # Publish motor command
        command_msg = String()
        command_msg.data = motor_command
        self.motor_publisher.publish(command_msg)
        self.get_logger().info(f'AI Agent sent command: {motor_command}')

    def process_with_ai(self, sensor_data):
        # Placeholder for AI logic
        # In a real implementation, this would use ML models
        if 'reading: 0' in sensor_data:
            return 'move_forward'
        elif 'reading: 1' in sensor_data:
            return 'turn_left'
        else:
            return 'idle'

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgent()
    rclpy.spin(ai_agent)
    ai_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Run the Complete System

```bash
cd ~/ros2_humanoid_ws
source install/setup.bash

# Terminal 1: Sensor publisher
ros2 run my_robot_pkg sensor_publisher

# Terminal 2: AI Agent
ros2 run my_robot_pkg ai_agent

# Terminal 3: Motor subscriber
ros2 run my_robot_pkg motor_subscriber
```

## Hands-on Exercise

Try modifying the URDF to add an ankle joint to the leg structure:
1. Add a `left_foot` link
2. Add a `left_ankle_joint` connecting the shin to the foot
3. Validate the URDF with `check_urdf`
4. Visualize the updated robot with `urdf_to_graphiz`

This exercise will help you understand how to extend the humanoid model with additional joints for more complex movement patterns.