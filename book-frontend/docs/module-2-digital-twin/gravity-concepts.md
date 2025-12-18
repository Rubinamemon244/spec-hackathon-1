---
sidebar_position: 8
---

# Gravity Simulation Concepts

## Learning Objectives

By the end of this section, you will be able to:
- Explain how gravity simulation works in Gazebo for humanoid robots
- Configure gravity parameters for different environments
- Understand the impact of gravity on humanoid robot dynamics
- Implement gravity-based control strategies

## Overview

Gravity simulation is fundamental to realistic humanoid robot behavior in digital twins. Unlike wheeled robots, humanoid robots must constantly fight against gravity to maintain balance and execute movements. Proper gravity simulation ensures that the digital twin accurately reflects the physical challenges faced by the real robot.

## Understanding Gravity in Simulation

### Gravity Vector Definition

In Gazebo, gravity is defined as a 3D vector that affects all objects in the simulation:

```
Gravity = (g_x, g_y, g_z) m/s²
```

Where:
- g_x: Gravity component in X direction (forward/backward)
- g_y: Gravity component in Y direction (left/right)
- g_z: Gravity component in Z direction (up/down)

### Standard Earth Gravity

The standard Earth gravity vector is:
```
Gravity = (0, 0, -9.81) m/s²
```

This means:
- No horizontal gravity component (0 m/s² in X and Y)
- 9.81 m/s² downward acceleration in Z direction
- Negative Z value indicates downward direction in Gazebo's coordinate system

## Gravity Configuration in Gazebo

### World File Configuration

Gravity is configured globally in the world file:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="gravity_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>

      <!-- Optional: Adjust physics parameters for gravity simulation -->
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>      <!-- Solver iterations for stability -->
          <sor>1.3</sor>          <!-- Successive over-relaxation -->
        </solver>
        <constraints>
          <cfm>0.0</cfm>          <!-- Constraint force mixing -->
          <erp>0.2</erp>          <!-- Error reduction parameter -->
        </constraints>
      </ode>
    </physics>

    <!-- Include your humanoid robot model -->
    <include>
      <uri>model://humanoid_robot</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### Runtime Gravity Modification

Gravity can be modified during simulation using Gazebo services:

```python
#!/usr/bin/env python3
"""
Example: Modifying gravity during simulation
"""

import rospy
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from geometry_msgs.msg import Vector3

def modify_gravity():
    """Modify gravity during simulation"""
    rospy.init_node('gravity_modifier')

    # Wait for the service to be available
    rospy.wait_for_service('/gazebo/set_physics_properties')

    try:
        # Get current physics properties
        get_physics = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        current_physics = get_physics()

        # Modify gravity (example: simulate moon gravity)
        moon_gravity = Vector3(0, 0, -1.62)  # Moon's gravity is ~1.62 m/s²

        # Set new physics properties
        set_physics = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

        response = set_physics(
            time_step=current_physics.time_step,
            max_update_rate=current_physics.max_update_rate,
            gravity=moon_gravity,
            ode_config=current_physics.ode_config
        )

        if response.success:
            rospy.loginfo(f"Gravity successfully changed to: {moon_gravity}")
        else:
            rospy.logerr("Failed to change gravity")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    modify_gravity()
```

## Gravity Effects on Humanoid Robots

### Balance and Posture

Gravity creates several challenges for humanoid robots:

- **Center of Mass**: Must be kept within support polygon
- **Postural Stability**: Requires active control to maintain upright position
- **Energy Consumption**: Fighting gravity requires continuous power
- **Gait Patterns**: Walking requires precise timing and force control

### Joint Load Distribution

Gravity creates different loads on various joints:

```python
# Example: Calculating gravity-induced joint torques
def calculate_gravity_torques(robot_state):
    """
    Calculate gravity-induced torques for humanoid robot
    robot_state: Current joint positions and robot configuration
    """
    # Simplified calculation for demonstration
    # In practice, this would use full kinematic and dynamic models

    gravity_torques = {}

    # For each joint, calculate the gravitational effect
    for joint_name, joint_angle in robot_state.joint_positions.items():
        # Calculate gravitational moment based on link positions
        gravity_torques[joint_name] = compute_gravity_torque(joint_name, joint_angle)

    return gravity_torques

def compute_gravity_torque(joint_name, angle):
    """
    Compute gravity torque for a specific joint
    This is a simplified example - real calculation is much more complex
    """
    # Simplified: torque = force * lever_arm * sin(angle)
    # where force = mass * gravity
    lever_arm = 0.5  # meters (example)
    mass = 5.0       # kg (example)

    gravity_torque = mass * 9.81 * lever_arm * math.sin(angle)
    return gravity_torque
```

## Different Gravity Environments

### Earth-like Gravity (9.81 m/s²)

Standard environment for most humanoid robot testing:

- **Walking Gait**: Natural walking patterns emerge
- **Balance Control**: Requires sophisticated balance algorithms
- **Manipulation**: Objects behave as expected in real world
- **Energy Requirements**: High power consumption to fight gravity

### Reduced Gravity (Moon: 1.62 m/s²)

Useful for testing space robotics or studying gravity effects:

- **Gait Changes**: Different walking patterns emerge
- **Balance**: Easier to maintain, less control required
- **Jumping**: Much higher jumps possible
- **Falling**: Slower, less dangerous falls

### Zero Gravity (0.0 m/s²)

Useful for space robotics or as a control baseline:

- **Locomotion**: Different movement strategies needed
- **Balance**: No balance required, but orientation control needed
- **Manipulation**: Objects float, different interaction required
- **Stability**: Different stability concepts apply

### Custom Gravity

For experimental purposes or special environments:

- **Negative Gravity**: Simulating levitation or anti-gravity
- **Directional Changes**: Testing robot response to gravity changes
- **Variable Gravity**: Simulating changing environments

## Implementing Gravity-Based Control

### Balance Control Algorithms

Gravity-aware balance control is essential:

```python
#!/usr/bin/env python3
"""
Gravity-based balance control for humanoid robot
"""

import rospy
import numpy as np
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

class GravityBalanceController:
    """Balance controller that accounts for gravity effects"""

    def __init__(self):
        rospy.init_node('gravity_balance_controller')

        # Subscribe to IMU data for balance feedback
        self.imu_sub = rospy.Subscribe('/robot/imu/data', Imu, self.imu_callback)

        # Subscribe to joint states
        self.joint_sub = rospy.Subscribe('/robot/joint_states', JointState, self.joint_callback)

        # Publishers for joint control
        self.joint_pubs = {}
        self.setup_joint_controllers()

        # Balance control parameters
        self.balance_kp = 10.0  # Proportional gain
        self.balance_kd = 2.0   # Derivative gain

        # Gravity vector (from IMU or known environment)
        self.gravity_vector = Vector3(0, 0, -9.81)

        # Robot state
        self.current_orientation = None
        self.current_angular_velocity = None
        self.joint_positions = {}
        self.joint_velocities = {}

    def setup_joint_controllers(self):
        """Setup joint command publishers"""
        joints_to_control = [
            'left_hip_pitch', 'left_knee', 'left_ankle_pitch',
            'right_hip_pitch', 'right_knee', 'right_ankle_pitch',
            'torso_pitch'
        ]

        for joint in joints_to_control:
            self.joint_pubs[joint] = rospy.Publisher(
                f'/robot/{joint}_position_controller/command',
                Float64,
                queue_size=10
            )

    def imu_callback(self, msg):
        """Process IMU data for balance control"""
        # Extract orientation from quaternion
        orientation = msg.orientation
        self.current_orientation = self.quaternion_to_euler(orientation)

        # Extract angular velocity
        self.current_angular_velocity = msg.angular_velocity

    def joint_callback(self, msg):
        """Update joint state"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.joint_velocities[name] = msg.velocity[i]

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles"""
        # Simplified conversion (in practice, use tf or similar)
        import math

        # Convert quaternion to roll, pitch, yaw
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

    def compute_balance_correction(self):
        """Compute balance correction based on gravity and orientation"""
        if self.current_orientation is None:
            return {}

        # Get current orientation errors
        roll, pitch, yaw = self.current_orientation

        # Compute desired joint angles to counteract gravity
        # This is a simplified example - real implementation is much more complex
        balance_corrections = {
            'left_hip_pitch': -pitch * self.balance_kp,
            'right_hip_pitch': -pitch * self.balance_kp,
            'torso_pitch': -pitch * self.balance_kp * 0.5,
            'left_ankle_pitch': pitch * self.balance_kp * 0.3,
            'right_ankle_pitch': pitch * self.balance_kp * 0.3
        }

        return balance_corrections

    def run(self):
        """Main control loop"""
        rate = rospy.Rate(100)  # 100 Hz

        while not rospy.is_shutdown():
            # Compute balance corrections based on gravity effects
            corrections = self.compute_balance_correction()

            # Apply corrections to joints
            for joint, correction in corrections.items():
                if joint in self.joint_pubs:
                    current_pos = self.joint_positions.get(joint, 0.0)
                    target_pos = current_pos + correction * 0.01  # Small correction step
                    self.joint_pubs[joint].publish(Float64(target_pos))

            rate.sleep()

if __name__ == '__main__':
    controller = GravityBalanceController()
    rospy.loginfo("Gravity balance controller started")
    controller.run()
```

## Gravity Simulation Best Practices

### Solver Configuration

For stable gravity simulation:

- **Increase solver iterations**: More iterations = more stable but slower
- **Adjust ERP (Error Reduction Parameter)**: Higher for more aggressive error correction
- **Tune CFM (Constraint Force Mixing)**: Lower for stiffer constraints

### Model Optimization

For efficient gravity simulation:

- **Use capsules for limbs**: Faster than complex meshes
- **Simplify collision geometry**: Balance accuracy with performance
- **Proper mass distribution**: Realistic for accurate gravity effects

### Validation Techniques

Verify gravity simulation accuracy:

1. **Drop Test**: Drop object and verify 9.81 m/s² acceleration
2. **Pendulum Test**: Create pendulum and verify natural frequency
3. **Balance Test**: Verify humanoid can maintain balance under gravity
4. **Energy Test**: Verify energy conservation in closed systems

## Summary

Gravity simulation is critical for realistic humanoid robot digital twins. Proper configuration and understanding of gravity effects enable accurate simulation of balance, locomotion, and manipulation tasks. The implementation of gravity-aware control systems is essential for stable robot operation in simulated environments.

## Next Steps

Continue to learn about collision detection and response in Gazebo physics simulation.

[← Previous: Physics Simulation with Gazebo](./physics-simulation-gazebo) | [Next: Collision Detection →](./collision-detection)