"""
Common utility functions for ROS 2 examples in the Humanoid Robotics module.

This module provides helper functions that are commonly used across
different ROS 2 examples and tutorials.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import math


def initialize_ros():
    """Initialize the ROS 2 client library."""
    if not rclpy.ok():
        rclpy.init()


def create_node(node_name):
    """Create and return a ROS 2 node."""
    initialize_ros()
    return Node(node_name)


def wait_for_message(node, msg_type, topic_name, timeout=5.0):
    """
    Wait for a single message on a topic.

    Args:
        node: The ROS 2 node
        msg_type: The message type (e.g., String, JointState)
        topic_name: The topic name to listen to
        timeout: Maximum time to wait in seconds

    Returns:
        The received message or None if timeout
    """
    msg = None
    sub = None

    def callback(message):
        nonlocal msg
        msg = message

    qos = QoSProfile(depth=1)
    sub = node.create_subscription(msg_type, topic_name, callback, qos)

    start_time = time.time()
    while msg is None and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    if sub:
        node.destroy_subscription(sub)

    return msg


def create_qos_profile(depth=10, reliability='reliable', durability='volatile'):
    """
    Create a QoS profile with specified parameters.

    Args:
        depth: History depth
        reliability: 'reliable' or 'best_effort'
        durability: 'volatile' or 'transient_local'

    Returns:
        QoSProfile object
    """
    qos = QoSProfile(depth=depth)

    if reliability == 'reliable':
        qos.reliability = QoSReliabilityPolicy.RELIABLE
    else:
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

    if durability == 'transient_local':
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    else:
        qos.durability = QoSDurabilityPolicy.VOLATILE

    return qos


def normalize_angle(angle):
    """
    Normalize an angle to the range [-pi, pi].

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in radians
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def calculate_joint_velocity(positions, dt):
    """
    Calculate joint velocities from position history.

    Args:
        positions: List of joint positions over time
        dt: Time step between measurements

    Returns:
        List of joint velocities
    """
    if len(positions) < 2:
        return [0.0] * len(positions[0]) if len(positions) > 0 else []

    velocities = []
    for i in range(len(positions[0])):
        vel = (positions[-1][i] - positions[-2][i]) / dt
        velocities.append(vel)

    return velocities


class JointStatePublisher:
    """
    A helper class to publish joint states easily.
    """

    def __init__(self, node, joint_names, publish_rate=50):
        """
        Initialize the joint state publisher.

        Args:
            node: The ROS 2 node
            joint_names: List of joint names to publish
            publish_rate: Rate at which to publish joint states (Hz)
        """
        self.node = node
        self.joint_names = joint_names
        self.publish_rate = publish_rate

        # Create publisher for joint states
        qos = create_qos_profile(depth=1)
        self.joint_state_pub = node.create_publisher(JointState, 'joint_states', qos)

        # Timer for publishing
        self.timer = node.create_timer(1.0/publish_rate, self.publish_joint_states)

        # Initialize joint state message
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = joint_names
        self.joint_state_msg.position = [0.0] * len(joint_names)
        self.joint_state_msg.velocity = [0.0] * len(joint_names)
        self.joint_state_msg.effort = [0.0] * len(joint_names)

    def set_joint_positions(self, positions):
        """Set the joint positions to publish."""
        if len(positions) == len(self.joint_names):
            self.joint_state_msg.position = list(positions)
        else:
            raise ValueError(f"Expected {len(self.joint_names)} positions, got {len(positions)}")

    def set_joint_velocities(self, velocities):
        """Set the joint velocities to publish."""
        if len(velocities) == len(self.joint_names):
            self.joint_state_msg.velocity = list(velocities)
        else:
            raise ValueError(f"Expected {len(self.joint_names)} velocities, got {len(velocities)}")

    def set_joint_efforts(self, efforts):
        """Set the joint efforts to publish."""
        if len(efforts) == len(self.joint_names):
            self.joint_state_msg.effort = list(efforts)
        else:
            raise ValueError(f"Expected {len(self.joint_names)} efforts, got {len(efforts)}")

    def publish_joint_states(self):
        """Publish the current joint states."""
        self.joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.joint_state_msg.header.frame_id = 'base_link'
        self.joint_state_pub.publish(self.joint_state_msg)


def shutdown_ros():
    """Shutdown the ROS 2 client library."""
    if rclpy.ok():
        rclpy.shutdown()


def main():
    """Example usage of the utility functions."""
    print("ROS 2 Helpers module - Common utilities for ROS 2 examples")
    print("Import this module to use the helper functions in your examples.")


if __name__ == '__main__':
    main()