---
title: Chapter 2 - Technical Middleware Layers and Humble Configuration
sidebar_label: Chapter 2 Technical Middleware Layers and Humble Configuration
id: ros2-architecture-setup
---

# Chapter 2: Technical Middleware Layers and Humble Configuration

## ROS 2 Humble Hawksbill Installation on Ubuntu 22.04

ROS 2 Humble Hawksbill is an LTS (Long Term Support) release that provides stability for robotic applications. This section provides a comprehensive guide for installation and initial configuration on Ubuntu 22.04.

### Prerequisites

Before installing ROS 2 Humble, ensure your system meets the following requirements:

- Ubuntu 22.04 LTS
- At least 4GB RAM (8GB+ recommended for development)
- At least 10GB free disk space
- Internet connection for package downloads

### Installation Steps

#### 1. Set up the ROS 2 Repository

```bash
# Add the ROS 2 GPG key
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

:::tip
**Hardware Optimization**: For Jetson Orin Nano, consider using the NVIDIA ROS 2 packages which are optimized for ARM64 architecture and include hardware acceleration support.
:::

#### 2. Install ROS 2 Packages

```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

This command installs the full desktop environment including RViz, Gazebo, and other development tools.

#### 3. Install Additional Development Tools

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
```

- `colcon-common-extensions`: Build system for ROS 2 packages
- `rosdep`: Dependency management tool
- `vcstool`: Version control system tool for managing multiple repositories

#### 4. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### Environment Setup

To use ROS 2, you need to source the setup script. Add the following line to your `~/.bashrc` file:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## ROS 2 Architecture Overview

### Nodes

Nodes are the fundamental execution units in ROS 2. Each node represents a single process that performs specific functions. Nodes communicate with each other through:

- **Topics**: Publish/subscribe communication pattern
- **Services**: Request/response communication pattern
- **Actions**: Goal-oriented communication with feedback

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics and Messages

Topics enable asynchronous communication between nodes using a publish/subscribe pattern. Messages are the data structures exchanged between nodes. ROS 2 provides standard message types and allows custom message definitions.

### Services

Services provide synchronous request/response communication. A service client sends a request to a service server, which processes the request and returns a response.

### Actions

Actions are used for long-running tasks that require feedback, goal management, and the ability to cancel operations.

## Creating a Workspace

A workspace is a directory where you modify and build ROS 2 packages. Here's how to create one:

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace (initially empty)
colcon build --packages-select
```

### Package Structure

A typical ROS 2 package includes:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata
├── src/                    # Source code
├── include/                # Header files
├── launch/                 # Launch files
├── config/                 # Configuration files
├── test/                   # Test files
└── README.md               # Documentation
```

### Building Packages

Use `colcon build` to compile your packages:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash
```

## Quality of Service (QoS) Settings

QoS settings allow you to specify communication requirements for different types of data:

- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient local
- **History**: Keep last N messages or keep all
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to detect if a publisher is alive

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Example: High-reliability for critical data
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

:::danger
**Hardware Considerations**: When deploying to Jetson Orin Nano, be mindful of QoS settings as overly strict requirements may impact performance on embedded systems.
:::

## Validated Code Examples

Here are additional code examples that have been validated for ROS 2 Humble:

### Complete Minimal Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Complete Minimal Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Server Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Summary

This chapter covered the essential steps for installing and configuring ROS 2 Humble on Ubuntu 22.04. We explored the fundamental architectural concepts including nodes, topics, services, and actions. We also discussed workspace creation and QoS settings, which are crucial for humanoid robot applications requiring real-time performance and reliability.