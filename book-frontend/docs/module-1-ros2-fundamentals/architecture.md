---
sidebar_position: 2
---

# ROS 2 Architecture

## Learning Objectives

By the end of this section, you will be able to:
- Describe the architecture of ROS 2 and its components
- Explain how ROS 2 differs from ROS 1 in terms of architecture
- Understand the DDS-based communication layer in ROS 2

## Overview

The architecture of ROS 2 is fundamentally different from ROS 1. While ROS 1 relied on a centralized master node for coordination, ROS 2 uses a decentralized architecture based on DDS (Data Distribution Service) for communication between nodes.

## Key Components

### DDS (Data Distribution Service)

DDS is an open standard for real-time systems that provides discovery, transport, and quality of service controls. In ROS 2, DDS implementations serve as the middleware that handles communication between nodes.

### Nodes

Nodes are the fundamental unit of execution in ROS 2. Each node contains the functionality required to perform specific tasks.

## Code Example

Here's a basic ROS 2 node example in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

ROS 2's architecture provides a more robust and scalable foundation for robotics development compared to ROS 1. The DDS-based middleware enables better real-time performance and improved support for distributed systems.

## Related Topics

- To understand how to implement communication patterns in ROS 2, see [Module 2: ROS 2 Communication](../module-1-ros2-communication/)
- For practical applications of ROS 2 architecture in humanoid robot modeling, see [Module 3: URDF Modeling](../module-1-urdf-modeling/)

## Next Steps

Continue to the next section to learn more about ROS 2's role in embodied intelligence systems.

[← Previous: Module Introduction](./) | [Next: ROS 2 in Embodied Intelligence →](./embodied-intelligence)