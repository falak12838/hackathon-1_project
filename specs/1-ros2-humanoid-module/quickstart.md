# Quickstart: ROS 2 Humanoid Module

**Created**: 2025-12-16
**Feature**: 1-ros2-humanoid-module
**Target**: AI students and developers entering humanoid robotics

## Prerequisites

Before starting with the ROS 2 Humanoid Module, ensure you have:

1. **Operating System**: Ubuntu 22.04 LTS (recommended) or a compatible Linux distribution
2. **ROS 2 Installation**: ROS 2 Humble Hawksbill installed following the official installation guide
3. **Docker**: For containerized examples (optional but recommended)
4. **Git**: For version control and example repositories
5. **Basic Python knowledge**: Understanding of Python 3 syntax and concepts

## Setup Steps

### 1. Install ROS 2 Humble Hawksbill

```bash
# Add ROS 2 GPG key and repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
sudo apt install python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

### 2. Create a Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_humanoid_ws/src
cd ~/ros2_humanoid_ws

# Source ROS 2 and build workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 3. Install Additional Dependencies

```bash
# Install simulation tools
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-xacro

# Install Python dependencies
pip3 install transforms3d
```

### 4. Verify Installation

```bash
# Test ROS 2 installation
ros2 topic list
# Should show an empty list (no active nodes yet)

# Test Python client
python3 -c "import rclpy; print('rclpy imported successfully')"
```

## Running Your First Example

### 1. Create a Simple Node

Create a file `~/ros2_humanoid_ws/src/my_first_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class HelloWorldNode(Node):
    def __init__(self):
        super().__init__('hello_world_node')
        self.get_logger().info('Hello ROS 2 World! This is your first humanoid robot node.')

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()

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

### 2. Make it Executable and Run

```bash
chmod +x ~/ros2_humanoid_ws/src/my_first_node.py
cd ~/ros2_humanoid_ws
source install/setup.bash
python3 ~/ros2_humanoid_ws/src/my_first_node.py
```

You should see the message "Hello ROS 2 World! This is your first humanoid robot node."

## Next Steps

1. **Chapter 1**: Proceed to learn about ROS 2 basics and DDS concepts
2. **Chapter 2**: Explore the communication model with nodes, topics, and services
3. **Chapter 3**: Understand robot structure with URDF and simulation readiness

## Troubleshooting

**Issue**: "command not found: ros2"
- **Solution**: Ensure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`

**Issue**: Python import errors
- **Solution**: Make sure your Python path includes ROS 2 libraries: `export PYTHONPATH=$PYTHONPATH:/opt/ros/humble/lib/python3.10/site-packages`

**Issue**: Gazebo won't start
- **Solution**: Install Gazebo packages: `sudo apt install ros-humble-gazebo-ros-pkgs`