---
sidebar_position: 7
---

# Physics Simulation with Gazebo

## Learning Objectives

By the end of this section, you will be able to:
- Understand the fundamentals of physics simulation in Gazebo
- Configure realistic physics parameters for humanoid robots
- Implement gravity, collision detection, and dynamic responses
- Create physics-based scenarios that mirror real-world behavior

## Overview

Physics simulation is the cornerstone of digital twin technology for humanoid robots. In Gazebo, realistic physics ensures that the virtual robot behaves similarly to its physical counterpart, making the digital twin a valuable tool for testing, training, and validation before deployment.

## Core Physics Concepts in Gazebo

### Gravity Simulation

Gazebo implements realistic gravity simulation that affects all objects in the environment:

- **Standard Earth Gravity**: (0, 0, -9.8) m/s² - negative Z indicates downward direction
- **Custom Gravity**: Can be adjusted for different planetary environments or special scenarios
- **Gravity-Free Zones**: Useful for simulating microgravity or testing balance algorithms

### Collision Detection

Accurate collision detection is essential for humanoid robot simulation:

- **Collision Shapes**: Boxes, spheres, capsules, and meshes for different body parts
- **Contact Materials**: Define friction, bounciness, and other surface properties
- **Collision Filtering**: Allow or ignore collisions between specific object types

### Dynamics Simulation

Dynamics determine how objects move and respond to forces:

- **Mass Properties**: Mass, center of mass, and inertia tensors
- **Joint Constraints**: Limits on movement for realistic joint behavior
- **Force Application**: External forces, torques, and actuator commands

## Configuring Physics for Humanoid Robots

### Setting Up a Physics Model

Humanoid robots require careful physics configuration to behave realistically:

```xml
<!-- Example: Humanoid robot link with physics properties -->
<link name="thigh">
  <inertial>
    <!-- Mass in kilograms -->
    <mass value="5.0" />
    <!-- Center of mass offset -->
    <origin xyz="0 0 -0.2" rpy="0 0 0" />
    <!-- Inertia tensor (calculated for cylindrical approximation) -->
    <inertia
      ixx="0.05" ixy="0" ixz="0"
      iyy="0.05" iyz="0" izz="0.01" />
  </inertial>

  <visual>
    <origin xyz="0 0 -0.2" rpy="0 0 0" />
    <geometry>
      <capsule radius="0.08" length="0.3" />
    </geometry>
  </visual>

  <collision>
    <origin xyz="0 0 -0.2" rpy="0 0 0" />
    <geometry>
      <capsule radius="0.08" length="0.3" />
    </geometry>
  </collision>
</link>
```

### Joint Configuration for Realism

Humanoid joints need realistic constraints:

```xml
<!-- Example: Knee joint with realistic limits -->
<joint name="knee_joint" type="revolute">
  <parent link="thigh" />
  <child link="shank" />
  <origin xyz="0 0 -0.4" rpy="0 0 0" />
  <axis xyz="0 1 0" />  <!-- Rotate around Y-axis (pitch) -->
  <limit
    lower="0"         <!-- Knee can't bend backwards -->
    upper="2.35"      <!-- Maximum bend ~135 degrees -->
    effort="100"      <!-- Maximum torque (N-m) -->
    velocity="5" />   <!-- Maximum angular velocity -->
  <dynamics
    damping="1.0"     <!-- Resistance to motion -->
    friction="0.1" /> <!-- Static friction coefficient -->
</joint>
```

## Gravity Simulation in Detail

### Understanding Gravity Parameters

Gravity in Gazebo is configured globally for each world:

```xml
<world name="humanoid_world">
  <physics type="ode">
    <!-- Standard Earth gravity -->
    <gravity>0 0 -9.8</gravity>

    <!-- Physics engine parameters -->
    <ode>
      <solver>
        <type>quick</type>
        <iters>10</iters>          <!-- Solver iterations -->
        <sor>1.3</sor>             <!-- Successive over-relaxation -->
      </solver>
      <constraints>
        <cfm>0.0</cfm>             <!-- Constraint force mixing -->
        <erp>0.2</erp>             <!-- Error reduction parameter -->
        <contact_max_correcting_vel>100</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>
</world>
```

### Gravity Effects on Humanoid Robots

Different gravity levels significantly affect humanoid robot behavior:

- **Earth Gravity (9.8 m/s²)**: Normal walking, running, and manipulation
- **Reduced Gravity (1.6 m/s²)**: Moon-like environment, affects gait and balance
- **Zero Gravity**: Space robotics, requires different locomotion strategies
- **Increased Gravity**: Testing robot strength and stability margins

## Collision Detection and Response

### Collision Geometry Types

Choose appropriate collision geometry for performance and accuracy:

- **Primitive Shapes**: Fastest computation (box, sphere, cylinder)
- **Capsules**: Good for limbs, faster than meshes
- **Convex Meshes**: More accurate but slower
- **Heightmaps**: For terrain collision

### Contact Material Properties

Configure realistic surface interactions:

```xml
<!-- Example: Contact material definition -->
<gazebo reference="foot">
  <collision>
    <surface>
      <friction>
        <ode>
          <mu>0.7</mu>            <!-- Coefficient of friction -->
          <mu2>0.7</mu2>          <!-- Secondary friction -->
          <fdir1>0 0 1</fdir1>    <!-- Friction direction -->
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.1</restitution_coefficient>  <!-- Bounciness -->
        <threshold>100000.0</threshold>                         <!-- Bounce threshold -->
      </bounce>
      <contact>
        <ode>
          <soft_cfm>0.0</soft_cfm>      <!-- Soft constraint force mixing -->
          <soft_erp>0.2</soft_erp>      <!-- Soft error reduction -->
          <kp>1000000000000.0</kp>      <!-- Contact stiffness -->
          <kd>1000000000000.0</kd>      <!-- Contact damping -->
        </ode>
      </contact>
    </surface>
  </collision>
</gazebo>
```

## Dynamics Simulation

### Inertia Tensor Calculations

Proper inertia tensors are crucial for realistic dynamics:

For a cylinder (approximation for limbs):
- Ixx = Iyy = m/12 * (3*r² + h²)
- Izz = m/2 * r²

Where:
- m = mass
- r = radius
- h = height

### Joint Dynamics

Configure joints for realistic movement:

```xml
<!-- Example: Hip joint with realistic dynamics -->
<joint name="hip_joint" type="revolute">
  <parent link="pelvis" />
  <child link="thigh" />
  <axis xyz="0 1 0" />
  <limit lower="-1.57" upper="1.57" effort="200" velocity="3" />
  <dynamics damping="2.0" friction="0.5" />
</joint>
```

## Practical Implementation

### Setting Up a Basic Physics Simulation

```python
#!/usr/bin/env python3
"""
Physics Simulation Example for Humanoid Robot
Demonstrates gravity, collision, and dynamics in Gazebo
"""

import rospy
import numpy as np
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64

class PhysicsSimulator:
    """Class to demonstrate physics simulation concepts"""

    def __init__(self):
        rospy.init_node('physics_simulator')

        # Service to set model states
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Publishers for joint control
        self.joint_pubs = {}
        joint_names = ['hip_joint', 'knee_joint', 'ankle_joint']

        for joint in joint_names:
            self.joint_pubs[joint] = rospy.Publisher(
                f'/robot/{joint}_position_controller/command',
                Float64,
                queue_size=1
            )

    def apply_gravity_effect(self, model_name):
        """Simulate gravity effects on robot model"""
        # Create a model state message
        model_state = ModelState()
        model_state.model_name = model_name

        # Apply gravity by setting downward acceleration
        # In simulation, this is handled by Gazebo physics
        # Here we demonstrate how to query and respond to gravity effects
        rospy.loginfo(f"Simulating gravity effects on {model_name}")

    def check_collisions(self):
        """Monitor for collision events"""
        # In practice, collision detection is handled by Gazebo
        # This method would process contact sensor data
        pass

    def simulate_balance(self):
        """Simulate balance control under gravity"""
        rate = rospy.Rate(100)  # 100 Hz control loop

        while not rospy.is_shutdown():
            # Read sensor data (simulated IMU, force/torque sensors)
            # Apply control algorithms to maintain balance
            # Publish joint commands to maintain upright position

            # Example: Simple balance control
            self.apply_balance_control()
            rate.sleep()

    def apply_balance_control(self):
        """Apply simple balance control to maintain upright position"""
        # This would implement actual balance control algorithms
        # For demonstration, we'll just send a stable joint configuration
        neutral_poses = {
            'hip_joint': 0.0,
            'knee_joint': 0.0,
            'ankle_joint': 0.0
        }

        for joint, pos in neutral_poses.items():
            if joint in self.joint_pubs:
                self.joint_pubs[joint].publish(Float64(pos))

if __name__ == '__main__':
    simulator = PhysicsSimulator()
    rospy.loginfo("Physics simulator initialized")

    # Demonstrate gravity effect
    simulator.apply_gravity_effect('humanoid_robot')

    # Start balance simulation
    simulator.simulate_balance()
```

## Advanced Physics Concepts

### Soft Body Simulation

For more realistic humanoid models, consider soft body physics:

- **Deformable Objects**: Muscles, skin, and soft tissues
- **Material Properties**: Elasticity, viscosity, and damping
- **Computational Cost**: Significantly higher than rigid body simulation

### Fluid Simulation

For underwater or special environment robots:

- **Buoyancy**: Upward force based on displaced fluid volume
- **Drag Forces**: Resistance based on velocity and shape
- **Viscosity**: Internal fluid resistance effects

## Validation and Testing

### Physics Accuracy Verification

Verify that your physics simulation matches real-world behavior:

1. **Compare with Physical Robot**: Test similar scenarios on both
2. **Energy Conservation**: Check that energy is conserved appropriately
3. **Stability Testing**: Verify stable behavior under various conditions
4. **Performance Monitoring**: Ensure simulation runs at acceptable speeds

### Common Physics Issues

- **Jittering**: Often caused by incorrect solver parameters
- **Tunneling**: Objects passing through each other (increase solver iterations)
- **Instability**: Usually due to high stiffness or low damping
- **Penetration**: Objects sinking into surfaces (adjust contact parameters)

## Summary

Physics simulation in Gazebo provides the foundation for realistic digital twins of humanoid robots. Proper configuration of gravity, collision detection, and dynamics ensures that the virtual robot behaves similarly to its physical counterpart. This enables safe testing, algorithm development, and training before deployment on expensive hardware.

## Next Steps

Continue to learn about specific gravity simulation concepts and their implementation for humanoid robots.

[← Previous: Code Standards](./code-standards) | [Next: Gravity Simulation →](./gravity-concepts)