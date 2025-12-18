---
sidebar_position: 4
---

# URDF Examples and Sample Files

## Learning Objectives

By the end of this section, you will be able to:
- Create complete URDF models for humanoid robot components
- Understand and modify existing URDF files
- Validate URDF models for correctness
- Use URDF tools for visualization and debugging

## Overview

This section provides practical examples of URDF files for humanoid robots, from simple single links to complete robot models. These examples demonstrate best practices and common patterns used in real humanoid robotics applications.

## Simple Link Example

Let's start with a simple link definition that could be part of a humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_link_example">
  <!-- Base link (root of the robot) -->
  <link name="base_link">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02" />
    </inertial>

    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.1" length="0.2" />
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0" />
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.1" length="0.2" />
      </geometry>
    </collision>
  </link>
</robot>
```

## Single Joint Example

A simple model with one joint connecting two links:

```xml
<?xml version="1.0"?>
<robot name="single_joint_example">
  <link name="base_link">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" />
      <geometry>
        <box size="0.2 0.2 0.1" />
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1" />
      </material>
    </visual>
  </link>

  <link name="moving_part">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 0.1" />
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" />
      <geometry>
        <cylinder radius="0.05" length="0.2" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1" />
      </material>
    </visual>
  </link>

  <joint name="single_joint" type="revolute">
    <parent link="base_link" />
    <child link="moving_part" />
    <origin xyz="0 0 0.05" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1" />
  </joint>
</robot>
```

## Complete Humanoid Arm Example

A more complex example showing a humanoid arm with multiple joints:

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">
  <!-- Shoulder base -->
  <link name="shoulder_base">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" />
      <geometry>
        <box size="0.05 0.1 0.05" />
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1" />
      </material>
    </visual>
  </link>

  <!-- Upper arm -->
  <link name="upper_arm">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" />
      <geometry>
        <capsule radius="0.04" length="0.2" />
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.5 1" />
      </material>
    </visual>
  </link>

  <!-- Shoulder joint -->
  <joint name="shoulder_yaw" type="revolute">
    <parent link="shoulder_base" />
    <child link="upper_arm" />
    <origin xyz="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2" />
  </joint>

  <!-- Elbow joint -->
  <link name="forearm">
    <inertial>
      <mass value="0.7" />
      <origin xyz="0 0 -0.1" />
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" />
      <geometry>
        <capsule radius="0.035" length="0.15" />
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.5 1" />
      </material>
    </visual>
  </link>

  <joint name="elbow_pitch" type="revolute">
    <parent link="upper_arm" />
    <child link="forearm" />
    <origin xyz="0 0 -0.3" />
    <axis xyz="0 1 0" />
    <limit lower="0" upper="2.35" effort="40" velocity="2" />
  </joint>

  <!-- Wrist joint -->
  <link name="hand">
    <inertial>
      <mass value="0.2" />
      <origin xyz="0 0 -0.03" />
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0005" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.03" />
      <geometry>
        <box size="0.08 0.06 0.06" />
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.5 1" />
      </material>
    </visual>
  </link>

  <joint name="wrist_pitch" type="revolute">
    <parent link="forearm" />
    <child link="hand" />
    <origin xyz="0 0 -0.15" />
    <axis xyz="0 1 0" />
    <limit lower="-0.78" upper="0.78" effort="20" velocity="3" />
  </joint>
</robot>
```

## URDF with Mesh Files

For more realistic humanoid models, you can include mesh files:

```xml
<?xml version="1.0"?>
<robot name="humanoid_with_meshes">
  <material name="white">
    <color rgba="1 1 1 1" />
  </material>

  <link name="head">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://humanoid_description/meshes/head.stl" scale="1 1 1" />
      </geometry>
      <material name="white" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02" />
    </inertial>
  </link>

  <!-- Additional links with meshes would follow -->
</robot>
```

## Complete Humanoid Torso Example

A more comprehensive example of a humanoid torso with multiple degrees of freedom:

```xml
<?xml version="1.0"?>
<robot name="humanoid_torso">
  <!-- Pelvis (base of the robot) -->
  <link name="pelvis">
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 0.05" />
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.15" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.05" />
      <geometry>
        <box size="0.2 0.2 0.1" />
      </geometry>
      <material name="dark_grey">
        <color rgba="0.3 0.3 0.3 1" />
      </material>
    </visual>
  </link>

  <!-- Spine -->
  <link name="torso">
    <inertial>
      <mass value="8.0" />
      <origin xyz="0 0 0.2" />
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.3" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" />
      <geometry>
        <capsule radius="0.12" length="0.3" />
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1" />
      </material>
    </visual>
  </link>

  <!-- Spine joint -->
  <joint name="torso_joint" type="fixed">
    <parent link="pelvis" />
    <child link="torso" />
    <origin xyz="0 0 0.1" />
  </joint>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="3.0" />
      <origin xyz="0 0 0.1" />
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.1" />
      <geometry>
        <sphere radius="0.12" />
      </geometry>
      <material name="skin_color">
        <color rgba="0.9 0.7 0.5 1" />
      </material>
    </visual>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0 0 0.45" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1" />
  </joint>

  <!-- Left shoulder -->
  <link name="left_shoulder">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0.05 0 0" />
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005" />
    </inertial>
    <visual>
      <origin xyz="0.05 0 0" />
      <geometry>
        <box size="0.1 0.08 0.08" />
      </geometry>
      <material name="dark_grey" />
    </visual>
  </joint>

  <joint name="left_shoulder_yaw" type="revolute">
    <parent link="torso" />
    <child link="left_shoulder" />
    <origin xyz="0.1 0.15 0.3" />
    <axis xyz="1 0 0" />
    <limit lower="-0.78" upper="1.57" effort="30" velocity="2" />
  </joint>
</robot>
```

## URDF Validation Tools

### Using check_urdf

To validate your URDF files, you can use ROS tools:

```bash
# Install urdfdom tools
sudo apt-get install liburdfdom-tools

# Check a URDF file
check_urdf /path/to/your/robot.urdf

# Parse and display the kinematic tree
urdf_to_graphiz /path/to/your/robot.urdf
```

### Python Validation Example

```python
from urdf_parser_py.urdf import URDF
from urdf_parser_py.xml_reflection.core import *

def validate_urdf(urdf_path):
    try:
        robot = URDF.from_xml_file(urdf_path)
        print(f"URDF loaded successfully: {robot.name}")
        print(f"Number of links: {len(robot.links)}")
        print(f"Number of joints: {len(robot.joints)}")

        # Check for common issues
        for joint in robot.joints:
            if joint.limit is not None:
                print(f"Joint {joint.name}: limits [{joint.limit.lower}, {joint.limit.upper}]")

        return True
    except Exception as e:
        print(f"URDF validation failed: {e}")
        return False
```

## URDF Best Practices

### 1. File Organization

```
robot_description/
├── urdf/
│   ├── robot.urdf (main file)
│   ├── head.urdf.xacro
│   ├── arms.urdf.xacro
│   └── legs.urdf.xacro
├── meshes/
│   ├── head.stl
│   ├── arm.dae
│   └── leg.obj
└── config/
    └── joint_limits.yaml
```

### 2. Use Xacro for Complex Models

For complex humanoid robots, use Xacro (XML Macros) to avoid repetition:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_xacro_example">
  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="arm_length" value="0.3" />
  <xacro:property name="arm_radius" value="0.04" />

  <!-- Macro for arm definition -->
  <xacro:macro name="arm" params="side parent *origin">
    <link name="${side}_upper_arm">
      <inertial>
        <mass value="1.0" />
        <origin xyz="0 0 -${arm_length/2}" />
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001" />
      </inertial>
      <visual>
        <origin xyz="0 0 -${arm_length/2}" />
        <geometry>
          <capsule radius="${arm_radius}" length="${arm_length}" />
        </geometry>
      </visual>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="${parent}" />
      <child link="${side}_upper_arm" />
      <xacro:insert_block name="origin" />
      <axis xyz="0 1 0" />
      <limit lower="-1.57" upper="1.57" effort="50" velocity="2" />
    </joint>
  </xacro:macro>

  <!-- Use the macro to create both arms -->
  <xacro:arm side="left" parent="torso">
    <origin xyz="0.1 0.15 0.3" rpy="0 0 0" />
  </xacro:arm>

  <xacro:arm side="right" parent="torso">
    <origin xyz="0.1 -0.15 0.3" rpy="0 0 0" />
  </xacro:arm>
</robot>
```

## Summary

These URDF examples demonstrate the essential elements needed to model humanoid robots. From simple single links to complete robot models, these patterns form the foundation of robot description in ROS. Understanding these examples will help you create accurate models of your own humanoid robots.

## Next Steps

Continue to learn about diagrams and visual explanations for URDF concepts.

[← Previous: Hardware Mapping](./hardware-mapping) | [Next: URDF Diagrams and Visualizations →](./urdf-diagrams)