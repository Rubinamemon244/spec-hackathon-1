---
sidebar_position: 2
---

# Nodes, Topics, and Services

## Learning Objectives

By the end of this section, you will be able to:
- Create and manage ROS 2 nodes using Python
- Implement topic-based communication between nodes
- Use services for request-response communication patterns
- Understand when to use topics vs services for different use cases

## Overview

Communication in ROS 2 is based on a distributed architecture where nodes communicate with each other through topics and services. Topics enable asynchronous message passing (publish/subscribe), while services provide synchronous request/response communication.

## Nodes

Nodes are the fundamental building blocks of ROS 2 applications. Each node contains the functionality required to perform specific tasks.

## Code Example: Basic Node

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('MyNode has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics

Topics enable asynchronous communication through a publish/subscribe model. Multiple nodes can publish to the same topic, and multiple nodes can subscribe to the same topic.

## Code Example: Publisher and Subscriber

```python
# Publisher example
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

# Subscriber example
class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

## Services

Services provide synchronous request/response communication between nodes.

## Summary

Understanding nodes, topics, and services is fundamental to developing ROS 2 applications for humanoid robotics. These communication patterns enable different robot components to coordinate effectively.

## Related Topics

- For foundational concepts about ROS 2 architecture, see [Module 1: ROS 2 Fundamentals](../module-1-ros2-fundamentals/architecture)
- To apply these communication patterns to humanoid robot models, see [Module 3: URDF Modeling](../module-1-urdf-modeling/)

## Next Steps

Continue to learn about Python rclpy concepts for implementing these communication patterns.

[← Previous: Module Introduction](./) | [Next: Python rclpy Concepts →](./python-rclpy)