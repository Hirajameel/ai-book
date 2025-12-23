# Quickstart Guide: Module 3 - The AI-Robot Brain

**Module**: Module 3: The AI-Robot Brain
**Created**: 2025-12-22
**Status**: In Progress

## Overview

This quickstart guide provides a fast path to get started with NVIDIA Isaac™, Isaac ROS, and ROS 2 Nav2 for hardware-accelerated robotics applications.

## Prerequisites

### Hardware Requirements
- NVIDIA RTX 40-series GPU (recommended) or Jetson AGX Orin/Nano
- 16GB+ RAM (32GB recommended)
- 500GB+ SSD storage for Isaac Sim assets

### Software Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- NVIDIA GPU drivers (535+)
- Isaac Sim 2023.1+
- Isaac ROS 3.0+

## Installation Steps

### 1. Install ROS 2 Humble
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros-iron/rpm/deps/gpg.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-argcomplete python3-colcon-common-extensions
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2. Install NVIDIA Isaac Sim
```bash
# Download Isaac Sim from NVIDIA Developer website
# Follow the installation instructions for your platform
# Verify installation:
cd ~/isaac-sim
bash runheadless.py -- --version
```

### 3. Install Isaac ROS
```bash
# Clone Isaac ROS repository
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git -b ros2
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git -b ros2
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox_image_pipeline.git -b ros2

# Build the packages
cd ~/colcon_ws
colcon build --symlink-install --packages-select isaac_ros_visual_slam isaac_ros_nvblox_image_pipeline
source install/setup.bash
```

### 4. Install Nav2 for Humanoids
```bash
# Install Nav2 packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-gui

# Clone humanoid-specific configurations
git clone https://github.com/ros-planning/navigation2_tutorials.git
```

## Quick Test: Isaac Sim + ROS Bridge

### 1. Launch Isaac Sim
```bash
cd ~/isaac-sim
bash runheadless.py -- --exec="omni.isaac.examples.ros2_bridge.ros2_camera_publisher"
```

### 2. Verify ROS Topics
```bash
# In a new terminal
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 topic list | grep image
```

### 3. Launch Visual SLAM
```bash
# Terminal 1: Launch Isaac ROS Visual SLAM
source ~/colcon_ws/install/setup.bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Terminal 2: Play a sample rosbag or use Isaac Sim camera
ros2 bag play sample_camera_data.bag
```

## Quick Test: Humanoid Navigation

### 1. Launch Nav2 with Isaac Sim
```bash
# Terminal 1: Launch Isaac Sim with humanoid robot
cd ~/isaac-sim
bash runheadless.py -- --exec="omni.isaac.examples.ros2_bridge.ros2_humanoid_nav"

# Terminal 2: Launch Nav2
source ~/colcon_ws/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

### 2. Send Navigation Goal
```bash
# Terminal 3: Send navigation goal
source /opt/ros/humble/setup.bash
ros2 run nav2_msgs send_goal.py 1.0 1.0 0.0
```

## Performance Verification

### 1. Check SLAM Performance
```bash
# Monitor SLAM performance
ros2 run isaac_ros_visual_slam visual_slam_node --ros-args --log-level info
```

### 2. Benchmark on Jetson
```bash
# On Jetson platform
sudo tegrastats --interval 1000  # Monitor power consumption
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

## Troubleshooting

### Common Issues

1. **CUDA Compatibility**: Ensure GPU drivers match Isaac ROS requirements
2. **Port Conflicts**: Check for port conflicts between Isaac Sim and ROS bridge
3. **Memory Issues**: Increase swap space for Isaac Sim on Jetson platforms

### Performance Tips

1. **Optimize for Jetson**: Use TensorRT for neural network acceleration
2. **Reduce Scene Complexity**: Simplify Isaac Sim scenes for real-time performance
3. **Monitor Power**: Use jetson_clocks and thermal management for sustained performance

## Next Steps

1. Complete Chapter 11: Introduction to NVIDIA Isaac™
2. Proceed to Chapter 12: Synthetic Data Generation
3. Advance to Chapter 13: Isaac ROS & Hardware Acceleration
4. Continue with humanoid navigation in Chapters 14-15

## Support Resources

- [NVIDIA Isaac Documentation](https://nvidia-isaac-ros.github.io/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Isaac Sim User Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/)

This quickstart provides the foundation for the complete Module 3 implementation, enabling rapid development and testing of hardware-accelerated robotics applications using NVIDIA Isaac ecosystem.