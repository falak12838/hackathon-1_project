# The Digital Twin: Gazebo & Unity Simulation

## Overview

A digital twin is a virtual replica of a physical system that simulates its behavior in real-time. In robotics, digital twins enable safe testing, validation, and training of robotic systems in simulated environments before deployment in the real world. This module covers two key simulation platforms: Gazebo for physics-based simulation and Unity for high-fidelity rendering and human-robot interaction.

## Learning Objectives

By the end of this module, students will be able to:
- Set up and configure Gazebo simulation environments
- Model physics, gravity, and collisions in simulated environments
- Create high-fidelity rendering environments in Unity
- Simulate various sensors (LiDAR, Depth Cameras, IMUs)
- Implement human-robot interaction scenarios in virtual environments
- Compare simulation fidelity with real-world performance

## Gazebo: Physics-Based Simulation

Gazebo is a robust physics simulation environment that provides realistic simulation of robots and their environments. It includes high-quality graphics rendering and supports complex physical interactions.

### Key Features of Gazebo

- **Physics Simulation**: Accurate modeling of rigid body dynamics, collisions, and contact forces
- **Sensor Simulation**: LiDAR, cameras, IMUs, GPS, and other sensors
- **Environment Modeling**: Complex world scenarios with lighting and weather effects
- **Plugin Architecture**: Extensible functionality through plugins
- **ROS Integration**: Native support for ROS and ROS 2 communication

### Setting Up a Gazebo Environment

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Your robot model -->
    <include>
      <uri>model://my_humanoid_robot</uri>
    </include>

    <!-- Custom objects -->
    <model name="table">
      <pose>1 0 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 0.1</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 0.1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <iyy>1</iyy>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Physics Parameters and Simulation

Gazebo allows precise control over physical properties:

- **Gravity**: Configurable gravitational force
- **Friction**: Static and dynamic friction coefficients
- **Damping**: Linear and angular damping for realistic motion
- **Contacts**: Collision detection and response parameters

## Unity: High-Fidelity Rendering and Interaction

Unity provides photorealistic rendering capabilities and sophisticated human-robot interaction frameworks. It's particularly valuable for testing perception algorithms and user interfaces.

### Unity Robotics Simulation Features

- **Photorealistic Rendering**: Advanced lighting and materials
- **XR Support**: Virtual and augmented reality integration
- **Animation Systems**: Complex character animation and IK
- **AI Integration**: Built-in ML-Agents for training
- **Physics Engine**: Advanced physics simulation

### Setting up Unity for Robotics

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    public string topicName = "/robot/cmd_vel";
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.instance;
    }

    void Update()
    {
        // Send commands to robot
        var command = new Twist();
        command.linear.x = Input.GetAxis("Vertical");
        command.angular.z = Input.GetAxis("Horizontal");

        ros.Send<Unity.Robotics.ROSTCPConnector.MessageTypes.GeometryMsgs.Twist>(topicName, command);
    }
}
```

## Sensor Simulation

Accurate sensor simulation is crucial for developing robust perception systems.

### LiDAR Simulation

LiDAR sensors in simulation provide 2D or 3D point cloud data similar to real sensors:

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libRayPlugin.so"/>
</sensor>
```

### Depth Camera Simulation

Depth cameras provide RGB-D data for 3D perception:

```xml
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so"/>
</sensor>
```

### IMU Simulation

Inertial Measurement Units provide orientation and acceleration data:

```xml
<sensor name="imu" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <topic>__default_topic__</topic>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <body_name>imu_link</body_name>
    <topicName>imu</topicName>
    <serviceName>imu_service</serviceName>
    <gaussianNoise>0.001</gaussianNoise>
    <updateRateHZ>100.0</updateRateHZ>
  </plugin>
</sensor>
```

## Practical Exercise: Creating a Simulated Environment

1. Design a world in Gazebo with obstacles and objects
2. Import the same environment into Unity for high-fidelity rendering
3. Implement sensor simulation for your humanoid robot
4. Test navigation and manipulation tasks in both simulators
5. Compare performance and identify differences between simulators

## Bridging Simulation to Reality

The "reality gap" refers to differences between simulation and real-world performance. Techniques to minimize this gap include:

- Domain randomization
- Sim-to-real transfer learning
- System identification and parameter tuning
- Validation against real-world data

## Chapter Summary

Digital twins using Gazebo and Unity provide essential tools for robotics development. Gazebo offers accurate physics simulation for testing algorithms, while Unity provides high-fidelity rendering for perception and interaction testing. Proper sensor simulation is crucial for developing robust robotic systems.

## Quiz Questions

1. What is the "reality gap" and how can it be minimized?
2. Compare the advantages of Gazebo vs Unity for robotics simulation.
3. Why is accurate sensor simulation important in robotics development?

## Exercises

1. Create a complex Gazebo world with multiple objects and obstacles
2. Implement a Unity scene with realistic lighting and materials
3. Simulate multiple sensor types on a humanoid robot model
4. Compare path planning results in simulation vs reality