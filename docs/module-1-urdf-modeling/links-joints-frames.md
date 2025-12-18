---
sidebar_position: 2
---

# Links, Joints, and Frames in URDF

## Learning Objectives

By the end of this section, you will be able to:
- Define and distinguish between links, joints, and frames in URDF
- Create basic URDF elements for humanoid robot components
- Understand the relationship between physical robot structure and URDF representation
- Implement proper coordinate systems for robot components

## Overview

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robot models. Understanding the fundamental elements—links, joints, and frames—is essential for modeling humanoid robots. These elements define the robot's physical structure, kinematic relationships, and coordinate systems.

## Links

Links represent rigid bodies in the robot. They are the basic building blocks of a URDF model that define the physical parts of the robot.

### Basic Link Structure

```xml
<link name="link_name">
  <inertial>
    <mass value="0.1" />
    <origin xyz="0 0 0" rpy="0 0 0" />
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.1 0.1 0.1" />
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.1 0.1 0.1" />
    </geometry>
  </collision>
</link>
```

### Link Components Explained

1. **Inertial**: Defines mass properties for physics simulation
   - `mass`: Mass of the link in kilograms
   - `origin`: Center of mass location and orientation
   - `inertia`: Inertia tensor values

2. **Visual**: Defines how the link appears in visualizations
   - `origin`: Position and orientation relative to link frame
   - `geometry`: Shape of the link (box, cylinder, sphere, mesh)
   - `material`: Visual appearance properties

3. **Collision**: Defines collision boundaries for physics simulation
   - Similar to visual but can be simplified for performance

## Joints

Joints define the kinematic and dynamic relationships between links. They specify how parts of the robot can move relative to each other.

### Joint Types

```xml
<!-- Revolute joint (rotational with limits) -->
<joint name="joint_name" type="revolute">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0 0 0.1" rpy="0 0 0" />
  <axis xyz="0 0 1" />
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
</joint>

<!-- Continuous joint (rotational without limits) -->
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0 0 0" rpy="0 0 0" />
  <axis xyz="0 0 1" />
</joint>

<!-- Prismatic joint (linear motion) -->
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0 0 0" rpy="0 0 0" />
  <axis xyz="1 0 0" />
  <limit lower="0" upper="0.1" effort="100" velocity="1" />
</joint>

<!-- Fixed joint (no motion) -->
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0 0 0.1" rpy="0 0 0" />
</joint>
```

### Joint Properties

1. **Parent/Child**: Defines the kinematic chain relationship
2. **Origin**: Position and orientation of the joint relative to the parent
3. **Axis**: Direction of motion for revolute and prismatic joints
4. **Limit**: For revolute and prismatic joints, defines motion constraints

## Frames

Frames define coordinate systems attached to links. They are crucial for spatial relationships and transformations in robotics.

### Frame Conventions

```xml
<!-- Each link has a default frame at its origin -->
<link name="base_link">
  <!-- The frame of this link is at the origin specified in joint definitions -->
</link>

<!-- Joint origin defines the transform between parent and child frames -->
<joint name="base_to_link1" type="fixed">
  <parent link="base_link" />
  <child link="link1" />
  <!-- This defines the position of link1's frame relative to base_link's frame -->
  <origin xyz="0 0 0.1" rpy="0 0 0" />
</joint>
```

## Complete Example: Simple Humanoid Leg

```xml
<?xml version="1.0"?>
<robot name="simple_leg">
  <!-- Base of the leg -->
  <link name="hip">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.1" />
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1" />
      </material>
    </visual>
  </link>

  <!-- Thigh link -->
  <link name="thigh">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 -0.2" />
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" />
      <geometry>
        <capsule radius="0.04" length="0.3" />
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1" />
      </material>
    </visual>
  </link>

  <!-- Hip joint -->
  <joint name="hip_joint" type="revolute">
    <parent link="hip" />
    <child link="thigh" />
    <origin xyz="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
  </joint>

  <!-- Shank (lower leg) link -->
  <link name="shank">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0 0 -0.15" />
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" />
      <geometry>
        <capsule radius="0.035" length="0.2" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1" />
      </material>
    </visual>
  </link>

  <!-- Knee joint -->
  <joint name="knee_joint" type="revolute">
    <parent link="thigh" />
    <child link="shank" />
    <origin xyz="0 0 -0.4" />
    <axis xyz="0 1 0" />
    <limit lower="0" upper="2.35" effort="100" velocity="1" />
  </joint>

  <!-- Foot link -->
  <link name="foot">
    <inertial>
      <mass value="0.5" />
      <origin xyz="0.05 0 -0.02" />
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002" />
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.02" />
      <geometry>
        <box size="0.15 0.08 0.04" />
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1" />
      </material>
    </visual>
  </link>

  <!-- Ankle joint -->
  <joint name="ankle_joint" type="revolute">
    <parent link="shank" />
    <child link="foot" />
    <origin xyz="0 0 -0.3" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1" />
  </joint>
</robot>
```

## URDF Best Practices for Humanoid Robots

### 1. Use Proper Inertial Properties
- Calculate realistic mass and inertia values
- Ensure center of mass is correctly positioned
- Use simplified collision geometry for performance

### 2. Follow Kinematic Chain Conventions
- Use consistent naming conventions
- Define joints in a logical order
- Ensure kinematic loops are properly handled

### 3. Consider Computational Performance
- Use simple geometric shapes for collision models
- Reduce the number of small details in collision models
- Use meshes only for visual elements when possible

## Practical Application in Humanoid Robotics

In humanoid robots, proper link and joint definitions are critical for:
- Forward and inverse kinematics calculations
- Dynamics simulation and control
- Visualization and debugging
- Integration with perception systems

## Summary

Links, joints, and frames form the foundation of URDF models. Understanding how to properly define these elements is essential for creating accurate and functional humanoid robot models. The relationships between these components determine how the robot moves and interacts with its environment.

## Next Steps

Continue to learn about hardware mapping and how URDF models connect to real robotic systems.

[← Previous: Module Introduction](./index) | [Next: Hardware Mapping →](./hardware-mapping)