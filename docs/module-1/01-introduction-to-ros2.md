# Introduction to ROS 2: The Robotic Nervous System

## Overview

ROS 2 (Robot Operating System 2) serves as the foundational middleware for modern robotics applications. Like the nervous system in biological organisms, ROS 2 provides the communication infrastructure that connects sensors, actuators, and computational processes in a robot.

## Learning Objectives

By the end of this module, students will be able to:
- Explain the role of middleware in robotic systems
- Identify and describe ROS 2 nodes, topics, and services
- Create and connect ROS 2 nodes using Python and C++
- Bridge AI agents to ROS controllers using rclpy
- Understand URDF (Unified Robot Description Format) for humanoid robots

## What is ROS 2?

ROS 2 is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Key Features of ROS 2

- **Distributed Computing**: Nodes can run on different machines and communicate seamlessly
- **Language Independence**: Support for multiple programming languages (C++, Python, Rust, etc.)
- **Real-time Support**: Deterministic behavior for time-critical applications
- **Security**: Built-in security features for safe robot operation
- **Middleware Abstraction**: Uses DDS (Data Distribution Service) for communication

## Nodes, Topics, and Services

### Nodes

A node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 program. Multiple nodes are managed in a single robot system.

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalNode()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics

Topics are named buses over which nodes exchange messages. Topics are transport agnostic, meaning that nodes don't need to know about the underlying transport mechanism (TCP, UDP, shared memory, etc.).

### Services

Services provide a request/reply communication pattern. Unlike topics which are asynchronous, services are synchronous and block until a response is received.

## Practical Exercise: Creating Your First ROS 2 Node

1. Create a new ROS 2 package:
```bash
ros2 pkg create --build-type ament_python my_robot_controller
```

2. Create a publisher node that sends sensor data
3. Create a subscriber node that processes this data
4. Test the communication between nodes

## Understanding URDF for Humanoids

URDF (Unified Robot Description Format) is an XML format for representing a robot model. For humanoid robots, URDF defines:

- Kinematic chains (limbs, joints)
- Physical properties (mass, inertia)
- Visual representation (meshes, colors)
- Collision properties

Example URDF snippet for a humanoid arm:
```xml
<joint name="shoulder_pan_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.0 0.2 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

## Bridge Between AI Agents and ROS Controllers

Modern AI agents can be integrated with ROS 2 using Python bridges. This allows AI models to send commands to the robot and receive sensor feedback.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import your_ai_model

class AIControllerBridge(Node):
    def __init__(self):
        super().__init__('ai_controller_bridge')

        # Subscriber for sensor data
        self.sensor_subscriber = self.create_subscription(
            String, 'sensor_data', self.sensor_callback, 10)

        # Publisher for commands
        self.command_publisher = self.create_publisher(
            String, 'robot_command', 10)

        # Initialize AI model
        self.ai_model = your_ai_model.load_model()

    def sensor_callback(self, msg):
        # Process sensor data through AI model
        command = self.ai_model.process(msg.data)

        # Publish command to robot
        cmd_msg = String()
        cmd_msg.data = command
        self.command_publisher.publish(cmd_msg)
```

## Chapter Summary

ROS 2 serves as the nervous system for robotic applications, enabling distributed computation and communication between different components. Understanding nodes, topics, and services is crucial for developing complex robotic systems. The integration of AI agents with ROS controllers opens up possibilities for intelligent robotic behavior.

## Quiz Questions

1. What is the primary difference between ROS 2 topics and services?
2. Explain the publish-subscribe communication pattern in ROS 2.
3. Why is URDF important for humanoid robotics?

## Exercises

1. Create a ROS 2 package with a publisher and subscriber node
2. Design a URDF model for a simple humanoid robot
3. Implement a bridge between a basic AI model and ROS 2 nodes