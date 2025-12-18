---
sidebar_position: 4
---

# Communication Examples and Patterns

## Learning Objectives

By the end of this section, you will be able to:
- Implement complex communication patterns in ROS 2 using Python
- Create multi-node communication systems
- Use advanced features like action servers and clients
- Debug communication issues in ROS 2 systems

## Overview

This section provides practical examples of ROS 2 communication patterns commonly used in humanoid robotics. These examples build upon the basic concepts and demonstrate how to implement more sophisticated communication architectures.

## Publisher-Subscriber with Custom Message Types

First, let's create a custom message for humanoid robot joint commands:

```python
# In a .msg file (e.g., JointCommand.msg):
# float64[] positions
# float64[] velocities
# float64[] efforts
# string[] joint_names
```

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
# Assuming you have generated the message type
# from your_package_msgs.msg import JointCommand

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(JointCommand, 'joint_commands', 10)

        # Timer to send commands periodically
        self.timer = self.create_timer(0.1, self.publish_joint_commands)

        self.get_logger().info('Joint Command Publisher Started')

    def publish_joint_commands(self):
        msg = JointCommand()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Example joint positions for a humanoid robot
        msg.joint_names = ['left_hip', 'left_knee', 'left_ankle',
                          'right_hip', 'right_knee', 'right_ankle']
        msg.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initial positions
        msg.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.efforts = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.joint_pub.publish(msg)
        self.get_logger().info(f'Published joint commands: {msg.positions}')
```

## Multi-Node Publisher-Subscriber System

Here's an example of a more complex system with multiple publishers and subscribers:

```python
# Sensor Data Aggregator Node
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

class SensorAggregator(Node):
    def __init__(self):
        super().__init__('sensor_aggregator')

        # Subscriptions for different sensor types
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)

        # Publisher for aggregated data
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        self.robot_state = {
            'imu_orientation': None,
            'joint_positions': {},
            'timestamp': None
        }

        self.get_logger().info('Sensor Aggregator Initialized')

    def imu_callback(self, msg):
        self.robot_state['imu_orientation'] = {
            'x': msg.orientation.x,
            'y': msg.orientation.y,
            'z': msg.orientation.z,
            'w': msg.orientation.w
        }
        self.robot_state['timestamp'] = msg.header.stamp
        self.publish_status()

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.robot_state['joint_positions'][name] = msg.position[i]
        self.publish_status()

    def publish_status(self):
        status_msg = String()
        status_msg.data = f"Robot Status - IMU: {self.robot_state['imu_orientation'] is not None}, Joints: {len(self.robot_state['joint_positions'])}"
        self.status_pub.publish(status_msg)
```

## Action Server for Long-Running Tasks

For humanoid robots, actions are often used for long-running tasks like walking or manipulation:

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from your_package_msgs.action import WalkToGoal  # Custom action definition

class WalkActionServer(Node):
    def __init__(self):
        super().__init__('walk_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            WalkToGoal,
            'walk_to_goal',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.get_logger().info('Walk Action Server Started')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = WalkToGoal.Feedback()
        result = WalkToGoal.Result()

        # Simulate walking progress
        for i in range(0, 101, 10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return result

            # Update feedback
            feedback_msg.current_distance = float(i) / 10.0
            goal_handle.publish_feedback(feedback_msg)

            # Sleep to simulate walking
            await rclpy.sleep(0.5)

        goal_handle.succeed()
        result.success = True
        result.final_position = goal_handle.request.target_pose
        self.get_logger().info('Goal succeeded')

        return result
```

## Action Client Example

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from your_package_msgs.action import WalkToGoal
from geometry_msgs.msg import Pose

class WalkActionClient(Node):
    def __init__(self):
        super().__init__('walk_action_client')
        self._action_client = ActionClient(self, WalkToGoal, 'walk_to_goal')

    def send_goal(self, target_x, target_y):
        goal_msg = WalkToGoal.Goal()
        goal_msg.target_pose = Pose()
        goal_msg.target_pose.position.x = target_x
        goal_msg.target_pose.position.y = target_y

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current distance: {feedback.current_distance}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
```

## Parameter Service for Dynamic Configuration

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rcl_interfaces.msg import Parameter, ParameterValue

class ParameterManager(Node):
    def __init__(self):
        super().__init__('parameter_manager')

        # Set up parameter services
        self.get_param_srv = self.create_service(
            GetParameters, 'get_robot_params', self.get_parameters_callback)
        self.set_param_srv = self.create_service(
            SetParameters, 'set_robot_params', self.set_parameters_callback)

        # Declare some parameters
        self.declare_parameter('walking_speed', 0.5)
        self.declare_parameter('step_height', 0.1)
        self.declare_parameter('max_joint_torque', 100.0)

    def get_parameters_callback(self, request, response):
        param_names = request.names
        if not param_names:  # If no names specified, return all
            param_names = [param.name for param in self._parameters.values()]

        response.values = []
        for name in param_names:
            param = self.get_parameter(name)
            response.values.append(param.value)

        return response

    def set_parameters_callback(self, request, response):
        response.results = []
        for param in request.parameters:
            try:
                self.set_parameters([param])
                response.results.append(True)  # Success
            except Exception as e:
                response.results.append(False)  # Failure
                self.get_logger().error(f'Failed to set parameter {param.name}: {e}')

        return response
```

## Communication Best Practices for Humanoid Robotics

### 1. Use Appropriate QoS Settings

```python
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

# For sensor data (frequent updates, can lose some messages)
sensor_qos = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST
)

# For critical commands (must be reliable)
command_qos = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST
)
```

### 2. Implement Proper Error Handling

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

class RobustCommunicationNode(Node):
    def __init__(self):
        super().__init__('robust_comm_node')

        # Create publisher with error handling
        try:
            self.publisher = self.create_publisher(String, 'topic', 10)
        except Exception as e:
            self.get_logger().error(f'Failed to create publisher: {e}')
            return

        # Handle potential exceptions in callbacks
        self.subscription = self.create_subscription(
            String, 'topic', self.safe_callback, 10)

    def safe_callback(self, msg):
        try:
            # Process message safely
            self.process_message(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')
```

## Summary

These examples demonstrate advanced communication patterns used in humanoid robotics applications. Understanding these patterns is crucial for building robust and efficient robotic systems that can coordinate multiple components effectively.

## Next Steps

With a solid understanding of communication patterns, you're ready to explore URDF modeling for humanoid robots.

[← Previous: Python rclpy Concepts](./python-rclpy)