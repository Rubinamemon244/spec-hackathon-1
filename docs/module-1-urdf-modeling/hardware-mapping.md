---
sidebar_position: 3
---

# Software-to-Hardware Mapping in URDF

## Learning Objectives

By the end of this section, you will be able to:
- Map URDF models to real hardware components in humanoid robots
- Understand the relationship between simulation and physical systems
- Configure URDF for integration with robot controllers
- Implement proper calibration between software and hardware

## Overview

The mapping between software URDF models and physical hardware is crucial for humanoid robotics. This connection enables the software to accurately represent the physical robot's structure, which is essential for control, simulation, and debugging. Proper mapping ensures that commands sent from software correspond correctly to actions in the physical world.

## URDF to Hardware Mapping Principles

### 1. Joint Mapping

The most critical aspect of software-to-hardware mapping is ensuring that URDF joints correspond to physical joints:

```xml
<!-- URDF definition -->
<joint name="left_knee_joint" type="revolute">
  <parent link="left_thigh" />
  <child link="left_shank" />
  <origin xyz="0 0 -0.4" rpy="0 0 0" />
  <axis xyz="0 1 0" />
  <limit lower="0.0" upper="2.35" effort="100.0" velocity="1.0" />
</joint>
```

```yaml
# Hardware mapping configuration (typically in a separate config file)
left_knee_joint:
  controller:
    type: position_controllers/JointPositionController
    joint: left_knee_joint
  hardware_interface:
    name: hardware_interface/PositionJointInterface
    joint_name: left_knee_joint
  calibration:
    offset: 0.05  # Calibration offset in radians
```

### 2. Transmission Elements

Transmissions define how actuators connect to joints in the URDF:

```xml
<!-- Transmission for a joint -->
<transmission name="left_knee_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_knee_joint">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_knee_motor">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### 3. Gazebo Integration

For simulation, additional elements map URDF to physics properties:

```xml
<!-- Gazebo-specific properties -->
<gazebo reference="left_thigh">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>1.0</kd>
</gazebo>

<gazebo reference="left_knee_joint">
  <implicitSpringDamper>1</implicitSpringDamper>
</gazebo>
```

## Hardware Interface Configuration

### ros2_control Integration

Modern ROS 2 systems use ros2_control for hardware abstraction:

```yaml
# Controller configuration file
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    left_leg_controller:
      type: position_controllers/JointGroupPositionController

left_leg_controller:
  ros__parameters:
    joints:
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint
```

```xml
<!-- URDF with ros2_control interface -->
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  <joint name="left_hip_joint">
    <command_interface name="position">
      <param name="min">-1.57</param>
      <param name="max">1.57</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

## Calibration and Validation

### 1. Zero Position Calibration

Calibrating the relationship between software and hardware zero positions:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')

        # Joint state subscription
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)

        # Calibration offset publisher
        self.cal_pub = self.create_publisher(
            Float64MultiArray, 'calibration_offsets', 10)

        # Store calibration data
        self.calibration_offsets = {}

    def joint_callback(self, msg):
        # Process joint states and apply calibration
        calibrated_positions = []
        for i, name in enumerate(msg.name):
            raw_pos = msg.position[i]
            offset = self.calibration_offsets.get(name, 0.0)
            calibrated_pos = raw_pos + offset
            calibrated_positions.append(calibrated_pos)
```

### 2. Kinematic Validation

Validating that the URDF model matches the physical robot:

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class KinematicValidator(Node):
    def __init__(self):
        super().__init__('kinematic_validator')

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically validate transforms
        self.timer = self.create_timer(1.0, self.validate_kinematics)

    def validate_kinematics(self):
        try:
            # Check if transforms exist between expected links
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'left_foot', rclpy.time.Time())

            # Validate transform is reasonable
            if abs(transform.transform.translation.z) > 2.0:
                self.get_logger().warn('Suspicious transform detected')

        except Exception as e:
            self.get_logger().error(f'Transform lookup failed: {e}')
```

## URDF for Different Hardware Platforms

### 1. Series Elastic Actuators (SEA)

For robots with series elastic actuators, the URDF should reflect the spring elements:

```xml
<link name="sea_output">
  <inertial>
    <mass value="0.1" />
    <origin xyz="0 0 0" />
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001" />
  </inertial>
</link>

<joint name="sea_spring_joint" type="revolute">
  <parent link="sea_input" />
  <child link="sea_output" />
  <origin xyz="0 0 0" />
  <axis xyz="0 0 1" />
  <!-- Spring characteristics -->
  <dynamics damping="0.1" friction="0.01" />
</joint>
```

### 2. Harmonic Drive Systems

For systems with harmonic drives, consider the gear ratio in control:

```xml
<transmission name="harmonic_drive_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint_with_harmonic_drive">
    <hardwareInterface>EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="harmonic_drive_motor">
    <hardwareInterface>EffortJointInterface</hardwareInterface>
    <mechanicalReduction>100</mechanicalReduction>  <!-- 100:1 gear ratio -->
  </actuator>
</transmission>
```

## Troubleshooting Common Mapping Issues

### 1. Sign Conventions

Ensure joint directions match between URDF and hardware:

```python
# Check if hardware needs direction inversion
def apply_direction_correction(self, urdf_position, joint_name):
    if joint_name in ['left_hip', 'right_knee']:
        return -urdf_position  # Invert direction
    return urdf_position
```

### 2. Unit Conversions

Ensure consistent units between software and hardware:

```python
# Example: If hardware uses degrees but URDF uses radians
def convert_units(self, value, from_units, to_units):
    if from_units == 'radians' and to_units == 'degrees':
        return value * 180.0 / 3.14159
    elif from_units == 'degrees' and to_units == 'radians':
        return value * 3.14159 / 180.0
    return value
```

## Practical Application in Humanoid Robotics

In humanoid robots, proper software-to-hardware mapping enables:
- Accurate forward and inverse kinematics
- Stable control algorithms
- Reliable simulation-to-reality transfer
- Proper safety limits enforcement

## Summary

Software-to-hardware mapping in URDF is fundamental to connecting your robot model to the physical world. Proper mapping ensures that control commands from your software correctly drive the physical robot, and that sensor feedback accurately reflects the robot's state. This connection is essential for all higher-level robotics functionality.

## Next Steps

With a solid understanding of URDF modeling, you're ready to explore diagrams and visual explanations for URDF concepts.

[← Previous: Links, Joints, and Frames](./links-joints-frames) | [Next: URDF Examples →](./urdf-examples)