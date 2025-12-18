---
sidebar_position: 3
---

# Python rclpy Concepts

## Learning Objectives

By the end of this section, you will be able to:
- Use the rclpy client library to create ROS 2 nodes in Python
- Implement publishers and subscribers using rclpy
- Create and use services and clients with rclpy
- Understand the lifecycle of ROS 2 nodes in Python

## Overview

rclpy is the Python client library for ROS 2. It provides the Python API that allows you to create ROS 2 nodes, publish and subscribe to topics, provide and use services, and more. Understanding rclpy is essential for developing ROS 2 applications in Python, which is particularly common in AI and robotics research.

## Basic Node Structure with rclpy

Every ROS 2 Python node using rclpy follows a standard structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # Initialize the parent Node class with a node name
        super().__init__('my_node_name')

        # Initialize node components here
        self.get_logger().info('Node has been initialized')

def main(args=None):
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of your node
    node = MyNode()

    # Keep the node running until shutdown
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating Publishers

Publishers allow nodes to send messages to topics. Here's how to create one:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # Create a publisher
        self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Create a timer to publish messages periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1
```

## Creating Subscribers

Subscribers allow nodes to receive messages from topics:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')

        # Create a subscription
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)  # QoS profile depth
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

## Working with Services

Services provide request-response communication:

### Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')

        # Create a service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')

        # Create a client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Parameters in rclpy

Parameters allow configuration of nodes:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('param_name', 'default_value')
        self.declare_parameter('frequency', 1.0)

        # Get parameter values
        param_value = self.get_parameter('param_name').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().double_value

        self.get_logger().info(f'Parameter value: {param_value}')
        self.get_logger().info(f'Frequency: {frequency}')
```

## Timers

Timers are used to execute callbacks periodically:

```python
import rclpy
from rclpy.node import Node

class TimerNode(Node):
    def __init__(self):
        super().__init__('timer_node')

        # Create a timer that calls the callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback executed {self.counter} times')
        self.counter += 1
```

## Quality of Service (QoS)

QoS profiles control the delivery of messages:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

class QoSNode(Node):
    def __init__(self):
        super().__init__('qos_node')

        # Create a custom QoS profile
        qos_profile = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.publisher = self.create_publisher(String, 'qos_topic', qos_profile)
```

## Practical Application in Humanoid Robotics

In humanoid robotics applications, rclpy concepts are used to:

- Control joint actuators through topic-based commands
- Receive sensor data from IMUs, cameras, and other sensors
- Coordinate between different control modules using services
- Configure robot parameters dynamically

## Summary

rclpy provides the Python interface to ROS 2's functionality. Understanding these concepts is crucial for implementing ROS 2 communication patterns in Python, which is the primary language for many AI and robotics applications.

## Next Steps

Continue to learn about more advanced communication patterns and how to implement runnable code examples.

[← Previous: Nodes, Topics, and Services](./nodes-topics-services) | [Next: Communication Examples →](./communication-examples)