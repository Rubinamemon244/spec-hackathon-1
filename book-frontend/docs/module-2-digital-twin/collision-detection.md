---
sidebar_position: 9
---

# Collision Detection and Response

## Learning Objectives

By the end of this section, you will be able to:
- Understand collision detection principles in Gazebo
- Configure collision properties for humanoid robot components
- Implement collision response strategies
- Analyze and handle collision events in digital twin systems

## Overview

Collision detection is essential for realistic humanoid robot simulation. It prevents parts of the robot from intersecting with each other or with the environment, and provides the physical interactions that make the digital twin behave like its real-world counterpart. Proper collision handling is crucial for both safety and realistic behavior in simulation.

## Collision Detection Fundamentals

### How Collision Detection Works

Gazebo uses a multi-stage approach to collision detection:

1. **Broad Phase**: Quick elimination of non-colliding pairs using bounding volumes
2. **Narrow Phase**: Precise collision detection between potentially colliding objects
3. **Contact Generation**: Creation of contact points and forces

### Collision Geometry Types

Different geometry types offer trade-offs between accuracy and performance:

- **Primitive Shapes**: Fastest computation (box, sphere, cylinder)
- **Capsules**: Good for limbs, faster than meshes with good accuracy
- **Convex Meshes**: More accurate but slower computation
- **Heightmaps**: For terrain collision detection

## Collision Configuration in Gazebo

### Basic Collision Properties

Collision properties are defined in the URDF/SDF model:

```xml
<link name="robot_link">
  <!-- Visual properties (for rendering) -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <capsule radius="0.05" length="0.5" />
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1" />
    </material>
  </visual>

  <!-- Collision properties (for physics simulation) -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <capsule radius="0.05" length="0.5" />
    </geometry>
  </collision>
</link>
```

### Advanced Collision Properties

Fine-tune collision behavior with surface properties:

```xml
<gazebo reference="robot_link">
  <collision>
    <surface>
      <!-- Friction properties -->
      <friction>
        <ode>
          <mu>0.7</mu>            <!-- Primary friction coefficient -->
          <mu2>0.7</mu2>          <!-- Secondary friction coefficient -->
          <fdir1>1 0 0</fdir1>    <!-- Friction direction -->
          <slip1>0.0</slip1>      <!-- Primary slip coefficient -->
          <slip2>0.0</slip2>      <!-- Secondary slip coefficient -->
        </ode>
      </friction>

      <!-- Bounce properties -->
      <bounce>
        <restitution_coefficient>0.1</restitution_coefficient>  <!-- Bounciness -->
        <threshold>100000.0</threshold>                         <!-- Bounce threshold -->
      </bounce>

      <!-- Contact properties -->
      <contact>
        <ode>
          <soft_cfm>0.0</soft_cfm>      <!-- Soft constraint force mixing -->
          <soft_erp>0.2</soft_erp>      <!-- Soft error reduction -->
          <kp>1000000000000.0</kp>      <!-- Contact stiffness -->
          <kd>1000000000000.0</kd>      <!-- Contact damping -->
          <max_vel>100.0</max_vel>      <!-- Maximum contact penetration velocity -->
          <min_depth>0.001</min_depth>  <!-- Penetration depth threshold -->
        </ode>
      </contact>
    </surface>
  </collision>
</gazebo>
```

## Collision Detection for Humanoid Robots

### Self-Collision Avoidance

Humanoid robots have many potential self-collision pairs:

```xml
<!-- Example: Self-collision filtering -->
<robot name="humanoid_robot">
  <!-- Disable self-collision between adjacent links -->
  <gazebo>
    <self_collide>false</self_collide>
  </gazebo>

  <!-- Or selectively disable specific collision pairs -->
  <gazebo reference="left_upper_arm">
    <collision>
      <surface>
        <contact>
          <collide_without_contact>1</collide_without_contact>
        </contact>
      </surface>
    </collision>
  </gazebo>
</robot>
```

### Foot-Ground Interaction

Critical for humanoid locomotion:

```xml
<gazebo reference="left_foot">
  <collision>
    <surface>
      <friction>
        <ode>
          <mu>0.8</mu>    <!-- High friction for good grip -->
          <mu2>0.8</mu2>
        </ode>
      </friction>
      <contact>
        <ode>
          <soft_erp>0.2</soft_erp>      <!-- Stable contact -->
          <soft_cfm>0.0001</soft_cfm>   <!-- Responsive contact -->
          <kp>1e9</kp>                  <!-- High stiffness -->
          <kd>1e6</kd>                  <!-- Appropriate damping -->
        </ode>
      </contact>
    </surface>
  </collision>
</gazebo>
```

## Collision Response Strategies

### Contact Force Calculation

Gazebo calculates contact forces based on material properties:

```python
#!/usr/bin/env python3
"""
Collision response analysis for humanoid robot
"""

import rospy
import numpy as np
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Float64

class CollisionResponseAnalyzer:
    """Analyze and respond to collision events"""

    def __init__(self):
        rospy.init_node('collision_response_analyzer')

        # Subscribe to contact sensor data
        self.contact_sub = rospy.Subscriber(
            '/robot/foot_contact_sensor/state',
            ContactsState,
            self.contact_callback
        )

        # Publishers for collision response
        self.response_pubs = {}
        self.setup_response_publishers()

        # Collision analysis data
        self.collision_history = []
        self.max_collision_force = 1000.0  # N

    def setup_response_publishers(self):
        """Setup publishers for collision response"""
        # Joint controllers for collision response
        joints = ['left_hip_pitch', 'right_hip_pitch', 'left_knee', 'right_knee']
        for joint in joints:
            self.response_pubs[joint] = rospy.Publisher(
                f'/robot/{joint}_position_controller/command',
                Float64,
                queue_size=10
            )

    def contact_callback(self, msg):
        """Process contact sensor data"""
        if len(msg.states) > 0:
            # Analyze collision forces
            total_force = self.calculate_total_force(msg.states)
            contact_points = len(msg.states)

            # Log collision event
            rospy.loginfo(f"Collision detected: {contact_points} contact points, "
                         f"total force: {total_force:.2f} N")

            # Store in history for trend analysis
            self.collision_history.append({
                'timestamp': rospy.get_time(),
                'total_force': total_force,
                'contact_points': contact_points,
                'contacts': msg.states
            })

            # Trigger collision response if force exceeds threshold
            if total_force > 500.0:  # 500N threshold
                self.handle_collision_response(msg.states)

    def calculate_total_force(self, contacts):
        """Calculate total force from all contact points"""
        total_force = 0.0
        for contact in contacts:
            # Sum forces from all contact points
            for wrench in contact.wrenches:
                force_magnitude = np.sqrt(
                    wrench.force.x**2 + wrench.force.y**2 + wrench.force.z**2
                )
                total_force += force_magnitude

        return total_force

    def handle_collision_response(self, contacts):
        """Handle collision response based on contact analysis"""
        # Analyze contact distribution
        contact_distribution = self.analyze_contact_distribution(contacts)

        # Determine appropriate response
        if contact_distribution['asymmetric']:
            # Asymmetric contact - adjust balance
            self.adjust_balance_response()
        elif contact_distribution['high_force']:
            # High force contact - protective response
            self.protective_response()
        else:
            # Normal contact - continue normal operation
            self.normal_response()

    def analyze_contact_distribution(self, contacts):
        """Analyze how contacts are distributed"""
        contact_info = {
            'left_side': 0,
            'right_side': 0,
            'front': 0,
            'back': 0,
            'total_force': 0
        }

        for contact in contacts:
            # Analyze contact positions
            pos = contact.contact_positions[0] if contact.contact_positions else None
            force = contact.wrenches[0].force if contact.wrenches else Vector3(0, 0, 0)

            if pos:
                force_magnitude = np.sqrt(force.x**2 + force.y**2 + force.z**2)
                contact_info['total_force'] += force_magnitude

                # Determine contact location
                if pos.x > 0:
                    contact_info['front'] += force_magnitude
                else:
                    contact_info['back'] += force_magnitude

                if pos.y > 0:
                    contact_info['left_side'] += force_magnitude
                else:
                    contact_info['right_side'] += force_magnitude

        # Determine if contact is asymmetric
        side_imbalance = abs(contact_info['left_side'] - contact_info['right_side'])
        is_asymmetric = side_imbalance > contact_info['total_force'] * 0.3

        # Determine if force is high
        is_high_force = contact_info['total_force'] > 800.0

        return {
            'asymmetric': is_asymmetric,
            'high_force': is_high_force,
            'distribution': contact_info
        }

    def adjust_balance_response(self):
        """Adjust robot balance in response to collision"""
        rospy.loginfo("Adjusting balance due to collision")

        # Send commands to adjust stance
        balance_adjustments = {
            'left_hip_pitch': 0.05,   # Slight adjustment
            'right_hip_pitch': 0.05,
            'left_ankle_pitch': -0.02,
            'right_ankle_pitch': -0.02
        }

        for joint, adjustment in balance_adjustments.items():
            if joint in self.response_pubs:
                self.response_pubs[joint].publish(Float64(adjustment))

    def protective_response(self):
        """Protective response for high-force collisions"""
        rospy.loginfo("Executing protective response")

        # Send commands to reduce impact
        protective_actions = {
            'left_knee': 0.2,      # Slight bend to absorb shock
            'right_knee': 0.2,
            'torso_pitch': -0.1    # Slight forward lean
        }

        for joint, action in protective_actions.items():
            if joint in self.response_pubs:
                self.response_pubs[joint].publish(Float64(action))

    def normal_response(self):
        """Normal response for typical contacts"""
        # Usually no special action needed
        pass

    def get_collision_statistics(self):
        """Get collision statistics for analysis"""
        if not self.collision_history:
            return None

        forces = [c['total_force'] for c in self.collision_history]
        avg_force = sum(forces) / len(forces)
        max_force = max(forces) if forces else 0
        collision_count = len(self.collision_history)

        return {
            'average_force': avg_force,
            'max_force': max_force,
            'collision_count': collision_count,
            'time_span': (self.collision_history[-1]['timestamp'] -
                         self.collision_history[0]['timestamp'])
        }

if __name__ == '__main__':
    analyzer = CollisionResponseAnalyzer()
    rospy.loginfo("Collision response analyzer started")

    # Run continuously
    rospy.spin()
```

## Collision Avoidance Strategies

### Proactive Collision Avoidance

Prevent collisions before they happen:

```python
#!/usr/bin/env python3
"""
Proactive collision avoidance for humanoid robot
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float64
import sensor_msgs.point_cloud2 as pc2

class ProactiveCollisionAvoider:
    """Proactively avoid collisions before they happen"""

    def __init__(self):
        rospy.init_node('proactive_collision_avoider')

        # Subscribe to sensor data
        self.laser_sub = rospy.Subscriber('/robot/laser_scan', LaserScan, self.laser_callback)
        self.pointcloud_sub = rospy.Subscriber('/robot/depth_cloud', PointCloud2, self.pc_callback)

        # Publishers for joint control
        self.joint_pubs = {}
        self.setup_joint_controllers()

        # Collision avoidance parameters
        self.safe_distance = 0.5  # meters
        self.avoidance_threshold = 1.0  # meters

        # Sensor data storage
        self.laser_data = None
        self.pointcloud_data = None

    def setup_joint_controllers(self):
        """Setup joint command publishers"""
        joints = ['left_hip_pitch', 'right_hip_pitch', 'left_ankle_roll', 'right_ankle_roll']
        for joint in joints:
            self.joint_pubs[joint] = rospy.Publisher(
                f'/robot/{joint}_position_controller/command',
                Float64,
                queue_size=10
            )

    def laser_callback(self, msg):
        """Process laser scan data for collision avoidance"""
        self.laser_data = msg
        self.check_for_potential_collisions()

    def pc_callback(self, msg):
        """Process point cloud data"""
        self.pointcloud_data = list(pc2.read_points(msg, field_names=("x", "y", "z")))

    def check_for_potential_collisions(self):
        """Check laser data for potential collisions"""
        if self.laser_data is None:
            return

        # Find minimum distance in forward sector
        ranges = self.laser_data.ranges
        if not ranges:
            return

        # Analyze forward 90-degree sector
        start_idx = len(ranges) // 2 - len(ranges) // 8  # -22.5 degrees
        end_idx = len(ranges) // 2 + len(ranges) // 8    # +22.5 degrees

        forward_ranges = ranges[start_idx:end_idx]
        valid_ranges = [r for r in forward_ranges if r > 0 and r < self.avoidance_threshold]

        if valid_ranges:
            min_distance = min(valid_ranges)
            if min_distance < self.safe_distance:
                self.execute_avoidance_maneuver(min_distance)

    def execute_avoidance_maneuver(self, distance):
        """Execute collision avoidance maneuver"""
        rospy.logwarn(f"Collision imminent! Distance: {distance:.2f}m")

        # Adjust foot position to prepare for impact
        # This is a simplified example - real implementation would be more sophisticated
        adjustments = {
            'left_ankle_roll': 0.1 if distance < 0.3 else 0.05,
            'right_ankle_roll': -0.1 if distance < 0.3 else -0.05,
            'left_hip_pitch': 0.05,
            'right_hip_pitch': 0.05
        }

        for joint, adjustment in adjustments.items():
            if joint in self.joint_pubs:
                self.joint_pubs[joint].publish(Float64(adjustment))

    def predict_collision_trajectory(self):
        """Predict potential collision based on robot motion"""
        # This would integrate with motion planning
        # to predict future collisions
        pass

if __name__ == '__main__':
    avoider = ProactiveCollisionAvoider()
    rospy.loginfo("Proactive collision avoider started")
    rospy.spin()
```

## Collision Detection Optimization

### Performance Considerations

Optimize collision detection for real-time performance:

- **Simplify collision geometry**: Use primitive shapes where possible
- **Adjust collision bounds**: Balance accuracy with performance
- **Use collision filtering**: Ignore unnecessary collision pairs
- **Tune physics parameters**: Optimize solver settings

### Accuracy vs. Performance Trade-offs

| Aspect | High Accuracy | High Performance |
|--------|---------------|------------------|
| Geometry | Meshes | Primitive shapes |
| Solver | More iterations | Fewer iterations |
| Contact | Detailed surface properties | Simplified properties |
| Update Rate | High frequency | Lower frequency |

## Advanced Collision Concepts

### Multi-Body Collision Handling

For complex humanoid robots with many links:

```xml
<!-- Example: Complex collision handling -->
<gazebo>
  <!-- Disable collisions between specific link pairs -->
  <disable_collisions between="torso" and="head"/>
  <disable_collisions between="left_upper_arm" and="torso" reason="Adjacent"/>
  <disable_collisions between="right_upper_arm" and="torso" reason="Adjacent"/>
</gazebo>
```

### Soft Contact Simulation

For more realistic interaction:

```xml
<gazebo reference="hand">
  <collision>
    <surface>
      <contact>
        <ode>
          <!-- Soft contact for more realistic hand-object interaction -->
          <soft_cfm>0.001</soft_cfm>
          <soft_erp>0.1</soft_erp>
          <!-- Lower stiffness for softer contact -->
          <kp>1e7</kp>
          <kd>1e5</kd>
        </ode>
      </contact>
    </surface>
  </collision>
</gazebo>
```

## Summary

Collision detection and response are critical for realistic humanoid robot simulation. Proper configuration of collision properties, understanding of collision response strategies, and optimization for performance ensure that the digital twin behaves appropriately when interacting with its environment. The implementation of both reactive and proactive collision handling systems enables safe and realistic robot operation in simulation.

## Next Steps

Continue to learn about dynamics simulation and joint constraints for humanoid robots.

[← Previous: Gravity Simulation](./gravity-concepts) | [Next: Dynamics Simulation →](./dynamics-simulation)