# Common ROS2 Message Examples for Isaac Integration

## Sensor Messages

### Camera Image
```python
from sensor_msgs.msg import Image

# Example camera image message from Isaac Sim
camera_msg = Image()
camera_msg.header.stamp = self.get_clock().now().to_msg()
camera_msg.header.frame_id = "camera_frame"
camera_msg.height = 480
camera_msg.width = 640
camera_msg.encoding = "rgb8"
camera_msg.is_bigendian = False
camera_msg.step = 640 * 3  # width * channels
# image data would be populated with actual pixel data
```

### PointCloud2
```python
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

# Example point cloud from LiDAR sensor
pointcloud_msg = PointCloud2()
pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
pointcloud_msg.header.frame_id = "lidar_frame"
pointcloud_msg.height = 1
pointcloud_msg.width = 1000  # number of points
pointcloud_msg.fields = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
pointcloud_msg.is_bigendian = False
pointcloud_msg.point_step = 16  # bytes per point
pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width
pointcloud_msg.is_dense = True
# data would be populated with actual point cloud data
```

## Navigation Messages

### Odometry
```python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from geometry_msgs.msg import Point, Quaternion

# Example odometry message from Isaac Sim
odom_msg = Odometry()
odom_msg.header.stamp = self.get_clock().now().to_msg()
odom_msg.header.frame_id = "odom"
odom_msg.child_frame_id = "base_link"

# Position and orientation
odom_msg.pose.pose.position = Point(x=1.0, y=2.0, z=0.0)
odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

# Velocity
odom_msg.twist.twist.linear.x = 0.5
odom_msg.twist.twist.angular.z = 0.1
```

### PoseStamped
```python
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion

# Example navigation goal
pose_msg = PoseStamped()
pose_msg.header.stamp = self.get_clock().now().to_msg()
pose_msg.header.frame_id = "map"

pose_msg.pose.position = Point(x=5.0, y=3.0, z=0.0)
pose_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)  # 90 degree rotation
```

## Isaac-Specific Messages

### IsaacGoal (Conceptual)
```python
# Conceptual Isaac-specific navigation goal
# This would be a custom message type for Isaac-specific navigation
isaac_goal_msg = IsaacGoal()  # Custom message type
isaac_goal_msg.header.stamp = self.get_clock().now().to_msg()
isaac_goal_msg.target_pose = pose_msg.pose
isaac_goal_msg.behavior_type = "explore"  # or "patrol", "follow", etc.
isaac_goal_msg.humanoid_specific_params = humanoid_params
```

## Performance Metrics
```python
# Conceptual performance metrics message
performance_msg = PerformanceMetrics()  # Custom message type
performance_msg.processing_time_ms = 15.2
performance_msg.fps = 30.0
performance_msg.gpu_utilization = 75.5
performance_msg.memory_usage_mb = 2048.0
```