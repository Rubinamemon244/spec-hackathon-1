# ROS2 Interfaces for Isaac ROS Educational Module

## Overview
This document outlines the ROS2 message and service interfaces that students will use in the Isaac ROS educational module. These interfaces enable communication between Isaac Sim, Isaac ROS perception nodes, and Nav2 navigation components.

## Standard ROS2 Message Types

### Sensor Messages (sensor_msgs)
- **Image**: Camera image data from Isaac Sim sensors
- **PointCloud2**: 3D point cloud data from LiDAR sensors
- **Imu**: Inertial measurement unit data
- **LaserScan**: 2D laser scan data
- **CameraInfo**: Camera calibration and intrinsic parameters

### Navigation Messages (nav_msgs)
- **Odometry**: Robot pose and velocity estimates
- **Path**: Sequence of poses for navigation plans
- **OccupancyGrid**: 2D map representation for navigation
- **GridCells**: Visualization of specific grid areas

### Geometry Messages (geometry_msgs)
- **PoseStamped**: Robot or target pose with timestamp
- **Twist**: Robot velocity commands (linear and angular)
- **Point**: 3D point in space
- **Quaternion**: 3D orientation representation

## Isaac ROS Specific Message Types

### Isaac ROS Vision Messages
- **FeatureArray**: Feature points for VSLAM
- **FeatureTracks**: Tracked feature points over time
- **StereoDisparityImage**: Stereo vision depth information
- **TensorList**: GPU-accelerated tensor data

### Isaac ROS Navigation Messages
- **IsaacGoal**: Isaac-specific navigation goal with additional metadata
- **IsaacPath**: Path with Isaac-specific optimization information
- **IsaacMap**: High-resolution map optimized for Isaac tools

## Key ROS2 Topics Used in Tutorials

### Isaac Sim to ROS2 Bridge
- **/isaac_sim/robot_state**: Robot joint states from simulation
- **/isaac_sim/sensor_data**: Sensor readings from Isaac Sim
- **/isaac_sim/clock**: Simulation time synchronization

### Isaac ROS Perception Topics
- **/isaac_ros/vslam/visual_slam/traj_map**: VSLAM trajectory map
- **/isaac_ros/vslam/visual_slam/pose**: Estimated camera pose
- **/isaac_ros/detectnet/detections**: Object detection results
- **/isaac_ros/segmentation/masks**: Semantic segmentation masks

### Nav2 Navigation Topics
- **/goal_pose**: Navigation goal commands
- **/navigate_to_pose/result**: Navigation completion status
- **/local_costmap/costmap**: Local obstacle map
- **/global_costmap/costmap**: Global obstacle map
- **/plan**: Planned navigation path
- **/cmd_vel**: Velocity commands to robot base

## ROS2 Services Used in Tutorials

### Navigation Services
- **/navigate_to_pose**: Send navigation goal to Nav2
- **/clear_costmap**: Clear obstacle maps
- **/get_costmap**: Retrieve current costmap
- **/load_map**: Load navigation map

### Perception Services
- **/isaac_ros/reset_vslam**: Reset VSLAM tracking
- **/isaac_ros/set_parameters**: Configure perception node parameters
- **/isaac_ros/toggle_processing**: Enable/disable perception processing

## Isaac ROS Action Interfaces

### Navigation Actions
- **/navigate_to_pose**: Action for sending navigation goals with feedback
  - Goal: NavigateToPoseGoal (target pose and metadata)
  - Result: NavigateToPoseResult (completion status)
  - Feedback: NavigateToPoseFeedback (progress updates)

### Perception Actions
- **/isaac_ros/feature_tracking**: Feature tracking control
  - Goal: FeatureTrackingGoal (tracking configuration)
  - Result: FeatureTrackingResult (tracking status)
  - Feedback: FeatureTrackingFeedback (tracking progress)

## Example Code Patterns for Tutorials

### Subscribing to Isaac ROS Topics
```python
import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class IsaacROSTutorialNode:
    def __init__(self):
        self.image_sub = self.create_subscription(
            Image, '/isaac_ros/camera/image_raw', self.image_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/isaac_ros/vslam/visual_slam/pose', self.pose_callback, 10)

    def image_callback(self, msg):
        # Process camera image from Isaac ROS
        pass

    def pose_callback(self, msg):
        # Process pose estimate from VSLAM
        pass
```

### Publishing to Navigation System
```python
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy.action

class NavigationTutorialNode:
    def send_navigation_goal(self, x, y, theta):
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.z = theta
        # Send to Nav2 system
        pass
```

## Quality of Service (QoS) Settings

### Recommended QoS for Isaac ROS
- **Sensor data**: Reliable delivery with sensor data durability
- **Navigation commands**: Reliable delivery with transient local durability
- **Status updates**: Best effort delivery for performance metrics
- **Parameter updates**: Reliable delivery with service durability

## Troubleshooting Common Interface Issues

### Topic Connection Problems
- Verify ROS_DOMAIN_ID consistency across all nodes
- Check that Isaac Sim bridge is running
- Ensure Isaac ROS nodes are properly launched
- Confirm network configuration for distributed systems

### Message Format Issues
- Validate message timestamps are synchronized
- Check frame_id consistency across transforms
- Verify data types match expected formats
- Confirm sensor calibration parameters are correct

### Performance Issues
- Monitor topic bandwidth usage
- Adjust message publishing rates as needed
- Use message filters for high-frequency topics
- Implement message throttling when appropriate