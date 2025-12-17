# ROS 2 Interface Contracts: Humanoid Robot Module

**Created**: 2025-12-16
**Feature**: 1-ros2-humanoid-module
**Domain**: Educational ROS 2 content for humanoid robotics

## Overview

This document defines the ROS 2 interface contracts for the educational examples in the Humanoid Robot Module. These contracts specify the message types, services, and actions that students will learn to use when building humanoid robot applications.

## Message Types

### 1. JointState Message Contract

**Package**: `sensor_msgs`
**Message Type**: `JointState`

**Purpose**: Represents the state of joints in a humanoid robot, including position, velocity, and effort.

**Fields**:
- `header`: std_msgs/Header - Timestamp and frame information
- `name`: string[] - Names of the joints
- `position`: float64[] - Joint positions in radians or meters
- `velocity`: float64[] - Joint velocities in rad/s or m/s
- `effort`: float64[] - Joint efforts in Nm or N

**Usage Context**: Used to publish joint states from robot controllers or subscribe to joint states for monitoring.

**Validation Rules**:
- Array lengths of name, position, velocity, and effort must match
- Position values must be within joint limits
- Joint names must match those defined in URDF

### 2. Twist Message Contract

**Package**: `geometry_msgs`
**Message Type**: `Twist`

**Purpose**: Represents linear and angular velocities for robot movement.

**Fields**:
- `linear`: geometry_msgs/Vector3 - Linear velocity (x, y, z)
- `angular`: geometry_msgs/Vector3 - Angular velocity (x, y, z)

**Usage Context**: Used for sending velocity commands to robot base controllers.

**Validation Rules**:
- Values should be within robot's physical capabilities
- Units are m/s for linear and rad/s for angular velocities

### 3. HumanoidCommand Message Contract

**Package**: `humanoid_msgs` (Educational package)
**Message Type**: `HumanoidCommand`

**Purpose**: Custom message for humanoid-specific commands.

**Fields**:
- `header`: std_msgs/Header - Timestamp and frame information
- `joint_commands`: JointCommand[] - Array of joint position/velocity commands
- `balance_mode`: bool - Whether to enable balance control
- `gait_type`: string - Type of walking gait to use
- `step_size`: float64 - Size of steps in meters

**Usage Context**: Used to send complex humanoid robot commands.

**Validation Rules**:
- Joint commands must match joints in robot's URDF
- Gait type must be one of: "walk", "trot", "crawl"
- Step size must be positive and within robot capabilities

## Service Types

### 1. SetJointTrajectory Service Contract

**Package**: `humanoid_control`
**Service Type**: `SetJointTrajectory`

**Purpose**: Service to set a trajectory for multiple joints over time.

**Request**:
- `trajectory`: trajectory_msgs/JointTrajectory - The trajectory to follow
- `execution_time`: float64 - Expected execution time in seconds

**Response**:
- `success`: bool - Whether the trajectory was accepted
- `message`: string - Additional information about success/failure
- `error_code`: int32 - Error code if success is false

**Usage Context**: Used to send complex movement sequences to humanoid robot controllers.

**Validation Rules**:
- Trajectory joint names must match robot's URDF
- Time stamps must be in ascending order
- Joint positions must be within joint limits

### 2. GetRobotState Service Contract

**Package**: `humanoid_control`
**Service Type**: `GetRobotState`

**Purpose**: Service to retrieve the current state of the humanoid robot.

**Request**: Empty

**Response**:
- `joint_states`: sensor_msgs/JointState - Current joint positions
- `base_pose`: geometry_msgs/Pose - Current base position and orientation
- `balance_state`: humanoid_msgs/BalanceState - Current balance status
- `timestamp`: builtin_interfaces/Time - When the state was captured

**Usage Context**: Used to query the current state of the robot for monitoring or planning.

**Validation Rules**:
- All returned values must represent actual robot state
- Timestamp must be recent (within 1 second)

## Action Types

### 1. WalkToGoal Action Contract

**Package**: `humanoid_navigation`
**Action Type**: `WalkToGoal`

**Purpose**: Action to make the humanoid robot walk to a specified goal position.

**Goal**:
- `target_pose`: geometry_msgs/Pose - Target position and orientation
- `max_duration`: builtin_interfaces/Duration - Maximum time to reach goal
- `tolerance`: float64 - Acceptable distance to target (meters)

**Result**:
- `success`: bool - Whether the goal was reached
- `final_pose`: geometry_msgs/Pose - Final robot position
- `path_length`: float64 - Actual distance traveled
- `execution_time`: builtin_interfaces/Duration - Time taken to complete

**Feedback**:
- `current_pose`: geometry_msgs/Pose - Current robot position
- `distance_to_goal`: float64 - Remaining distance to target
- `status`: string - Current navigation status

**Usage Context**: Used to command the robot to walk to specific locations.

**Validation Rules**:
- Target pose must be in reachable area
- Tolerance must be positive
- Robot must maintain balance during navigation

## Topic Namespaces

### Standard Topics

- `/joint_states` - Joint state publisher topic
- `/cmd_vel` - Velocity command topic for base movement
- `/tf` and `/tf_static` - Transform topics for robot state
- `/rosout` - Logging topic

### Humanoid-Specific Topics

- `/humanoid/joint_commands` - Joint command topic
- `/humanoid/balance_state` - Balance state feedback
- `/humanoid/gait_commands` - Gait pattern commands
- `/humanoid/foot_pressure` - Foot pressure sensor data

## Quality of Service (QoS) Profiles

### Default Profile for Sensors
- Reliability: Reliable
- Durability: Volatile
- History: Keep Last (n=10)
- Deadline: None

### Default Profile for Commands
- Reliability: Reliable
- Durability: Volatile
- History: Keep Last (n=1)
- Deadline: 100ms

## Error Handling

### Standard Error Codes
- `0`: Success
- `1`: Invalid parameters
- `2`: Joint limit exceeded
- `3`: Balance lost
- `4`: Collision detected
- `5`: Timeout
- `6`: Hardware fault