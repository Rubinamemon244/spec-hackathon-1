---
sidebar_position: 6
---

# Code Snippet Standards for Simulation Examples

## Learning Objectives

By the end of this section, you will be able to:
- Apply proper syntax highlighting for different programming languages
- Format code snippets for simulation examples
- Include appropriate comments and explanations
- Structure code examples for educational effectiveness

## Overview

Code snippets are essential for demonstrating simulation concepts in digital twin applications. Proper formatting, syntax highlighting, and documentation help students understand how to implement simulation techniques in their own projects.

## Language-Specific Standards

### Python for Gazebo and ROS Integration

Python examples for Gazebo and ROS integration should follow these standards:

```python
#!/usr/bin/env python3
# Example: Basic Gazebo simulation controller

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import math

class GazeboController:
    """
    Basic controller for Gazebo simulation
    Demonstrates communication with Gazebo through ROS
    """

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('gazebo_controller')

        # Publishers for joint commands
        self.joint_pub = rospy.Publisher(
            '/robot/joint_position_controller/command',
            Float64,
            queue_size=10
        )

        # Subscribers for sensor data
        self.laser_sub = rospy.Subscriber(
            '/robot/laser_scan',
            LaserScan,
            self.laser_callback
        )

        # Control parameters
        self.target_position = 0.0
        self.max_velocity = 1.0

    def laser_callback(self, data):
        """
        Process laser scan data from simulated sensor
        data: LaserScan message from Gazebo simulation
        """
        # Find minimum distance in forward direction
        forward_range = data.ranges[len(data.ranges)//2]  # Front of robot

        if forward_range < 1.0:  # If obstacle within 1 meter
            self.target_position = 0.0  # Stop or reverse
        else:
            self.target_position = math.sin(rospy.get_time())  # Move in sine pattern

    def run(self):
        """Main control loop"""
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Publish target position
            self.joint_pub.publish(Float64(self.target_position))
            rate.sleep()

if __name__ == '__main__':
    controller = GazeboController()
    controller.run()
```

### C# for Unity Robotics

Unity C# examples should follow these standards:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class UnityRobotController : MonoBehaviour
{
    // Robot configuration parameters
    [Header("Robot Configuration")]
    public float maxSpeed = 2.0f;
    public float rotationSpeed = 90.0f;  // degrees per second
    public Transform robotBase;

    // Sensor simulation
    [Header("Sensor Simulation")]
    public Camera rgbCamera;
    public Camera depthCamera;
    public Light mainLight;

    // ROS communication
    private ROSConnection ros;
    private string robotTopic = "unity_robot_control";

    // Start is called before the first frame update
    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<RobotCommandMsg>(robotTopic);
    }

    // Update is called once per frame
    void Update()
    {
        // Process user input for robot control
        float moveVertical = Input.GetAxis("Vertical");
        float moveHorizontal = Input.GetAxis("Horizontal");

        // Apply movement to robot
        Vector3 movement = new Vector3(moveHorizontal, 0.0f, moveVertical);
        movement = transform.TransformDirection(movement);
        movement *= maxSpeed * Time.deltaTime;

        robotBase.Translate(movement);

        // Rotate robot based on horizontal input
        float rotation = rotationSpeed * moveHorizontal * Time.deltaTime;
        robotBase.Rotate(0, rotation, 0);
    }

    // LateUpdate is called after Update
    void LateUpdate()
    {
        // Update sensor data based on robot position
        UpdateSensorData();
    }

    void UpdateSensorData()
    {
        // Update camera positions based on robot movement
        if (rgbCamera != null)
        {
            rgbCamera.transform.position = robotBase.position + new Vector3(0, 0.5f, 0);
            rgbCamera.transform.rotation = robotBase.rotation;
        }

        if (depthCamera != null)
        {
            depthCamera.transform.position = robotBase.position + new Vector3(0, 0.5f, 0);
            depthCamera.transform.rotation = robotBase.rotation;
        }
    }
}
```

### XML for Gazebo and URDF

XML examples for Gazebo world files and URDF models:

```xml
<?xml version="1.0"?>
<!-- Example: Humanoid robot model with sensors -->
<robot name="humanoid_robot">
  <!-- Base link - pelvis -->
  <link name="base_link">
    <inertial>
      <mass value="10.0" />
      <origin xyz="0 0 0" />
      <inertia
        ixx="0.1" ixy="0" ixz="0"
        iyy="0.1" iyz="0" izz="0.1" />
    </inertial>

    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <capsule radius="0.15" length="0.2" />
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0" />
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <capsule radius="0.15" length="0.2" />
      </geometry>
    </collision>
  </link>

  <!-- Head link -->
  <link name="head">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 0.1" />
      <inertia
        ixx="0.01" ixy="0" ixz="0"
        iyy="0.01" iyz="0" izz="0.01" />
    </inertial>

    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
      <material name="skin_color">
        <color rgba="0.9 0.7 0.5 1.0" />
      </material>
    </visual>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link" />
    <child link="head" />
    <origin xyz="0 0 0.3" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1" />
  </joint>

  <!-- LiDAR sensor -->
  <gazebo reference="head">
    <sensor name="laser_scanner" type="ray">
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
        <topicName>/robot/laser_scan</topicName>
        <frameName>head</frameName>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Comment Standards

### Inline Comments

Use inline comments sparingly and only for complex logic:

```python
# Good: Explains why a specific value is used
threshold = 0.8  # 80% confidence threshold based on empirical testing

# Avoid: Simply restating what the code does
x = x + 1  # Add 1 to x
```

### Block Comments

Use block comments for complex algorithms or important explanations:

```python
"""
Complex motion planning algorithm that considers:
1. Obstacle avoidance using potential fields
2. Dynamic balance maintenance for humanoid robots
3. Energy-efficient trajectory optimization
"""
def plan_motion_with_balance(robot_state, target_position):
    # Implementation here
    pass
```

## Code Organization

### File Structure

Organize simulation code with clear sections:

```python
#!/usr/bin/env python3
"""
Simulation Controller for Humanoid Robot Digital Twin

This module provides control and simulation interfaces for
a humanoid robot digital twin, including sensor simulation
and physics-based interactions.
"""

# Standard library imports
import math
import time

# Third-party imports
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

# Local imports
from simulation.physics import PhysicsEngine
from simulation.sensors import SensorSimulator


class DigitalTwinController:
    """Main controller class for the digital twin"""

    def __init__(self):
        """Initialize the digital twin controller"""
        # Initialize components
        self.physics = PhysicsEngine()
        self.sensors = SensorSimulator()

        # Setup ROS communication
        self.setup_ros()

    def setup_ros(self):
        """Setup ROS publishers and subscribers"""
        # Implementation here
        pass

    def run_simulation(self):
        """Main simulation loop"""
        # Implementation here
        pass


if __name__ == '__main__':
    controller = DigitalTwinController()
    controller.run_simulation()
```

## Educational Considerations

### Progressive Complexity

Start with simple examples and build complexity gradually:

```python
# Simple: Basic sensor reading
def read_laser_simple():
    ranges = get_laser_data()
    return min(ranges)  # Get closest obstacle

# Intermediate: With error handling
def read_laser_intermediate():
    try:
        ranges = get_laser_data()
        valid_ranges = [r for r in ranges if r > 0 and r < 30.0]
        return min(valid_ranges) if valid_ranges else float('inf')
    except Exception as e:
        rospy.logwarn(f"Laser reading error: {e}")
        return float('inf')

# Advanced: With filtering and analysis
def read_laser_advanced():
    ranges = get_laser_data()
    # Apply noise filtering
    filtered_ranges = apply_noise_filter(ranges)
    # Analyze for obstacles in different directions
    forward_obstacles = analyze_sector(filtered_ranges, -0.5, 0.5)
    return forward_obstacles
```

### Error Handling

Include appropriate error handling in examples:

```python
def connect_to_gazebo():
    """Connect to Gazebo simulation with error handling"""
    max_retries = 5
    retry_delay = 1.0

    for attempt in range(max_retries):
        try:
            # Attempt to connect to Gazebo
            gazebo_connection = establish_connection()
            rospy.loginfo("Successfully connected to Gazebo")
            return gazebo_connection
        except ConnectionError as e:
            rospy.logwarn(f"Connection attempt {attempt + 1} failed: {e}")
            if attempt < max_retries - 1:
                rospy.sleep(retry_delay)
                retry_delay *= 1.5  # Exponential backoff
            else:
                rospy.logerr("Failed to connect to Gazebo after all retries")
                raise

    return None
```

## Summary

Proper code snippet standards enhance the educational value of simulation examples. Clear syntax highlighting, appropriate comments, and well-structured examples help students understand and implement simulation concepts effectively. Following these standards ensures consistency and educational effectiveness across all digital twin content.

## Next Steps

Continue to the next phase of implementation, focusing on the specific user stories for physics simulation, Unity environments, and sensor simulation.

[← Previous: Sensor Concepts](./sensor-concepts) | [Next: Physics Simulation with Gazebo →](./physics-simulation-gazebo)