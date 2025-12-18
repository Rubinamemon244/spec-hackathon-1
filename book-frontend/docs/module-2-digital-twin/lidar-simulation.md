---
sidebar_position: 19
---

# LiDAR Simulation for Humanoid Robots

## Learning Objectives

By the end of this section, you will be able to:
- Configure realistic LiDAR sensors in Gazebo simulation environments
- Understand the principles of LiDAR operation and point cloud generation
- Implement LiDAR noise models that match real-world sensor characteristics
- Validate LiDAR simulation accuracy for digital twin applications

## Overview

LiDAR (Light Detection and Ranging) sensors are crucial for humanoid robots operating in complex environments. They provide accurate 3D spatial information that enables navigation, mapping, obstacle detection, and localization. In digital twin environments, realistic LiDAR simulation is essential for training perception algorithms and validating navigation systems before deployment on physical robots.

## LiDAR Fundamentals

### How LiDAR Works

LiDAR sensors operate by emitting laser pulses and measuring the time it takes for the light to reflect off objects and return to the sensor. This "time of flight" measurement allows the calculation of distance to objects in the environment.

Key operational principles:
- **Laser Emission**: Short laser pulses are emitted at specific angles
- **Time Measurement**: Precise measurement of round-trip time for each pulse
- **Distance Calculation**: Distance = (speed of light × time) / 2
- **Point Cloud Generation**: Collection of 3D points representing the environment

### LiDAR Specifications and Parameters

Different LiDAR sensors have varying specifications that affect their performance:

- **Range**: Minimum and maximum detection distances (typically 0.1m to 100m+)
- **Accuracy**: Measurement precision (often ±2cm to ±5cm)
- **Resolution**: Angular resolution of measurements (0.1° to 2°)
- **Field of View**: Horizontal and vertical coverage (e.g., 360° × 30°)
- **Update Rate**: Frequency of complete scans (5-20 Hz typical)
- **Power**: Laser power and eye safety classification

## LiDAR Configuration in Gazebo

### Basic LiDAR Sensor Configuration

```xml
<!-- Example: Basic LiDAR configuration -->
<sensor name="laser_2d" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>        <!-- Number of rays per scan -->
        <resolution>1</resolution>     <!-- Resolution of sampling -->
        <min_angle>-1.570796</min_angle> <!-- -90 degrees in radians -->
        <max_angle>1.570796</max_angle>  <!-- 90 degrees in radians -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>                 <!-- Minimum range (meters) -->
      <max>30.0</max>                 <!-- Maximum range (meters) -->
      <resolution>0.01</resolution>   <!-- Range resolution (meters) -->
    </range>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate>      <!-- Update rate (Hz) -->
  <visualize>true</visualize>        <!-- Visualize in GUI -->
</sensor>
```

### Advanced LiDAR Configuration

```xml
<!-- Example: Advanced LiDAR with realistic noise and plugins -->
<sensor name="advanced_lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>1080</samples>       <!-- High-resolution scanning -->
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle> <!-- Full 360-degree coverage -->
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.05</min>               <!-- Very close minimum range -->
      <max>50.0</max>                <!-- Long-range capability -->
      <resolution>0.001</resolution>  <!-- High precision -->
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>          <!-- 2cm standard deviation -->
    </noise>
  </ray>
  <always_on>1</always_on>
  <update_rate>15</update_rate>
  <visualize>true</visualize>

  <!-- LiDAR plugin configuration -->
  <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
    <topicName>/robot/laser_scan</topicName>
    <frameName>laser_frame</frameName>
    <min_range>0.1</min_range>
    <max_range>50.0</max_range>
    <gaussian_noise>0.02</gaussian_noise>
  </plugin>
</sensor>
```

## Multi-Layer LiDAR Configuration

### 3D LiDAR with Multiple Layers

```xml
<!-- Example: Multi-layer LiDAR (Velodyne-style) -->
<sensor name="velodyne_vlp16" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>1800</samples>       <!-- High horizontal resolution -->
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>         <!-- 16 vertical channels -->
        <resolution>1</resolution>
        <min_angle>-0.2618</min_angle> <!-- -15 degrees -->
        <max_angle>0.2618</max_angle>  <!-- +15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.2</min>                 <!-- Minimum range -->
      <max>100.0</max>                <!-- Maximum range -->
      <resolution>0.001</resolution>   <!-- Range precision -->
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.008</stddev>          <!-- Very precise ranging -->
    </noise>
  </ray>
  <always_on>1</always_on>
  <update_rate>10</update_rate>
  <visualize>true</visualize>

  <plugin name="velodyne_driver" filename="libgazebo_ros_velodyne_laser.so">
    <topicName>/robot/velodyne_points</topicName>
    <frameName>velodyne</frameName>
    <min_range>0.9</min_range>
    <max_range>130.0</max_range>
    <gaussian_noise>0.008</gaussian_noise>
  </plugin>
</sensor>
```

## LiDAR Simulation Implementation

### LiDAR Data Processing Pipeline

```python
#!/usr/bin/env python3
"""
LiDAR data processing pipeline for humanoid robot digital twin
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import struct

class LiDARProcessor:
    """Processes LiDAR data for humanoid robot applications"""

    def __init__(self):
        rospy.init_node('lidar_processor')

        # LiDAR configuration parameters
        self.angle_min = -np.pi/2      # -90 degrees
        self.angle_max = np.pi/2       # 90 degrees
        self.angle_increment = np.pi / 180  # 1 degree resolution
        self.range_min = 0.1           # Minimum range (meters)
        self.range_max = 30.0          # Maximum range (meters)
        self.scan_time = 0.1           # Time between scans (seconds)

        # Publishers
        self.processed_scan_pub = rospy.Publisher('/robot/laser_processed', LaserScan, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/robot/obstacles', PointCloud2, queue_size=10)

        # Subscribers
        self.raw_scan_sub = rospy.Subscriber('/robot/laser_scan', LaserScan, self.scan_callback)

        # Processing parameters
        self.obstacle_threshold = 1.0  # Distance threshold for obstacles (meters)
        self.min_cluster_points = 3    # Minimum points to form an obstacle cluster

        # Obstacle detection state
        self.obstacles = []
        self.last_scan_time = rospy.Time.now()

    def scan_callback(self, scan_msg):
        """Process incoming LiDAR scan data"""
        # Process the raw scan
        processed_scan = self.process_scan(scan_msg)

        # Detect obstacles
        obstacles = self.detect_obstacles(processed_scan)

        # Publish processed data
        self.processed_scan_pub.publish(processed_scan)
        self.publish_obstacles(obstacles)

    def process_scan(self, scan_msg):
        """Apply filtering and preprocessing to LiDAR scan"""
        # Create a copy of the scan message
        processed_scan = LaserScan()
        processed_scan.header = Header()
        processed_scan.header.stamp = rospy.Time.now()
        processed_scan.header.frame_id = scan_msg.header.frame_id

        # Copy scan parameters
        processed_scan.angle_min = scan_msg.angle_min
        processed_scan.angle_max = scan_msg.angle_max
        processed_scan.angle_increment = scan_msg.angle_increment
        processed_scan.time_increment = scan_msg.time_increment
        processed_scan.scan_time = scan_msg.scan_time
        processed_scan.range_min = scan_msg.range_min
        processed_scan.range_max = scan_msg.range_max

        # Apply filtering to the ranges
        processed_scan.ranges = self.filter_ranges(scan_msg.ranges)

        # Process intensities if available
        if scan_msg.intensities:
            processed_scan.intensities = self.filter_intensities(scan_msg.intensities)
        else:
            processed_scan.intensities = []

        return processed_scan

    def filter_ranges(self, ranges):
        """Apply noise filtering and outlier removal to range data"""
        # Convert to numpy array for processing
        ranges_array = np.array(ranges)

        # Remove infinite and NaN values
        ranges_array = np.where(np.isfinite(ranges_array), ranges_array, self.range_max)

        # Apply median filtering to reduce noise
        filtered_ranges = self.median_filter(ranges_array, window_size=3)

        # Apply range validation
        filtered_ranges = np.clip(filtered_ranges, self.range_min, self.range_max)

        return filtered_ranges.tolist()

    def median_filter(self, data, window_size=3):
        """Apply median filter to reduce noise"""
        if len(data) < window_size:
            return data

        # Pad the data to handle edges
        pad_width = window_size // 2
        padded_data = np.pad(data, pad_width, mode='edge')

        # Apply sliding window median
        filtered_data = np.zeros_like(data)
        for i in range(len(data)):
            window = padded_data[i:i + window_size]
            filtered_data[i] = np.median(window)

        return filtered_data

    def filter_intensities(self, intensities):
        """Apply filtering to intensity data"""
        intensities_array = np.array(intensities)

        # Remove invalid intensity values
        intensities_array = np.where(intensities_array > 0, intensities_array, 0)

        # Apply smoothing filter
        if len(intensities_array) >= 3:
            # Simple moving average
            smoothed = np.convolve(intensities_array, np.ones(3)/3, mode='same')
            intensities_array = smoothed

        return intensities_array.tolist()

    def detect_obstacles(self, scan_msg):
        """Detect obstacles from LiDAR scan data"""
        obstacles = []

        # Convert scan to Cartesian coordinates
        angles = np.linspace(
            scan_msg.angle_min,
            scan_msg.angle_max,
            len(scan_msg.ranges)
        )

        for i, range_val in enumerate(scan_msg.ranges):
            if self.range_min <= range_val <= self.obstacle_threshold:
                # Calculate Cartesian coordinates
                x = range_val * np.cos(angles[i])
                y = range_val * np.sin(angles[i])

                # Add point if it's within obstacle range
                obstacles.append([x, y, 0.0])  # z=0 for 2D obstacles

        return obstacles

    def publish_obstacles(self, obstacles):
        """Publish detected obstacles as PointCloud2"""
        if not obstacles:
            return

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "laser_frame"

        # Create PointCloud2 message
        fields = [
            point_cloud2.PointField('x', 0, point_cloud2.PointField.FLOAT32, 1),
            point_cloud2.PointField('y', 4, point_cloud2.PointField.FLOAT32, 1),
            point_cloud2.PointField('z', 8, point_cloud2.PointField.FLOAT32, 1)
        ]

        obstacle_cloud = point_cloud2.create_cloud(header, fields, obstacles)
        self.obstacle_pub.publish(obstacle_cloud)

    def run(self):
        """Main processing loop"""
        rospy.loginfo("LiDAR processor started")
        rospy.spin()

if __name__ == '__main__':
    processor = LiDARProcessor()
    processor.run()
```

## Point Cloud Processing

### 3D Point Cloud Operations

```python
#!/usr/bin/env python3
"""
3D Point Cloud processing for LiDAR data
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import pcl  # Python PCL library

class PointCloudProcessor:
    """Processes 3D point cloud data from LiDAR sensors"""

    def __init__(self):
        rospy.init_node('pointcloud_processor')

        # Subscribers
        self.cloud_sub = rospy.Subscriber('/robot/velodyne_points', PointCloud2, self.cloud_callback)

        # Publishers
        self.filtered_pub = rospy.Publisher('/robot/cloud_filtered', PointCloud2, queue_size=10)
        self.ground_pub = rospy.Publisher('/robot/cloud_ground', PointCloud2, queue_size=10)
        self.obstacles_pub = rospy.Publisher('/robot/cloud_obstacles', PointCloud2, queue_size=10)

        # Processing parameters
        self.voxel_leaf_size = 0.1  # Voxel grid leaf size (meters)
        self.ground_height_threshold = 0.1  # Ground segmentation threshold
        self.cluster_tolerance = 0.5  # Euclidean clustering tolerance
        self.min_cluster_size = 10   # Minimum points for object cluster
        self.max_cluster_size = 1000 # Maximum points for object cluster

    def cloud_callback(self, cloud_msg):
        """Process incoming point cloud data"""
        try:
            # Convert PointCloud2 to numpy array
            points_list = []
            for point in point_cloud2.read_points(cloud_msg, skip_nans=True):
                points_list.append([point[0], point[1], point[2]])

            if not points_list:
                return

            points_array = np.array(points_list, dtype=np.float32)

            # Convert to PCL point cloud for processing
            pcl_cloud = pcl.PointCloud_PointXYZ()
            pcl_cloud.from_array(points_array)

            # Apply voxel grid filtering
            filtered_cloud = self.apply_voxel_filter(pcl_cloud)

            # Segment ground plane
            ground_cloud, obstacle_cloud = self.segment_ground_plane(filtered_cloud)

            # Cluster obstacles
            clustered_obstacles = self.cluster_obstacles(obstacle_cloud)

            # Publish results
            self.publish_cloud(filtered_cloud, self.filtered_pub, cloud_msg.header.frame_id)
            self.publish_cloud(ground_cloud, self.ground_pub, cloud_msg.header.frame_id)
            self.publish_cloud(clustered_obstacles, self.obstacles_pub, cloud_msg.header.frame_id)

        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")

    def apply_voxel_filter(self, cloud):
        """Apply voxel grid filtering for downsampling"""
        # Create voxel grid filter
        vg = cloud.make_voxel_grid_filter()
        vg.set_leaf_size(
            self.voxel_leaf_size,
            self.voxel_leaf_size,
            self.voxel_leaf_size
        )
        return vg.filter()

    def segment_ground_plane(self, cloud):
        """Segment ground plane using RANSAC"""
        # Create segmentation object
        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(self.ground_height_threshold)

        # Segment the largest planar component
        inliers, coefficients = seg.segment()

        # Extract ground and obstacles
        ground_cloud = cloud.extract(inliers, negative=False)
        obstacle_cloud = cloud.extract(inliers, negative=True)

        return ground_cloud, obstacle_cloud

    def cluster_obstacles(self, cloud):
        """Cluster obstacles using Euclidean clustering"""
        # Create extraction indices
        tree = cloud.make_kdtree()

        # Create Euclidean cluster extraction
        ec = cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(self.cluster_tolerance)
        ec.set_MinClusterSize(self.min_cluster_size)
        ec.set_MaxClusterSize(self.max_cluster_size)
        ec.set_SearchMethod(tree)

        # Extract clusters
        cluster_indices = ec.Extract()

        # Combine all cluster points
        clustered_points = []
        for j, indices in enumerate(cluster_indices):
            for idx in indices:
                point = cloud[idx]
                clustered_points.append([point[0], point[1], point[2]])

        if clustered_points:
            clustered_cloud = pcl.PointCloud_PointXYZ()
            clustered_cloud.from_list(clustered_points)
            return clustered_cloud
        else:
            # Return empty cloud if no clusters found
            empty_cloud = pcl.PointCloud_PointXYZ()
            empty_cloud.from_list([])
            return empty_cloud

    def publish_cloud(self, pcl_cloud, publisher, frame_id):
        """Convert PCL cloud to ROS message and publish"""
        if pcl_cloud.size == 0:
            return

        # Convert PCL to numpy array
        points = pcl_cloud.to_array()

        # Create header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        # Create PointCloud2 message
        fields = [
            point_cloud2.PointField('x', 0, point_cloud2.PointField.FLOAT32, 1),
            point_cloud2.PointField('y', 4, point_cloud2.PointField.FLOAT32, 1),
            point_cloud2.PointField('z', 8, point_cloud2.PointField.FLOAT32, 1)
        ]

        ros_cloud = point_cloud2.create_cloud(header, fields, points)
        publisher.publish(ros_cloud)

    def run(self):
        """Main processing loop"""
        rospy.loginfo("Point cloud processor started")
        rospy.spin()

if __name__ == '__main__':
    processor = PointCloudProcessor()
    processor.run()
```

## LiDAR-Based SLAM

### Simultaneous Localization and Mapping

```python
#!/usr/bin/env python3
"""
LiDAR-based SLAM for humanoid robot digital twin
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point
from tf import TransformListener
import tf
from scipy.spatial.distance import cdist

class LiDARSLAM:
    """LiDAR-based SLAM implementation for digital twin"""

    def __init__(self):
        rospy.init_node('lidar_slam')

        # Map parameters
        self.map_resolution = 0.05  # 5cm per cell
        self.map_width = 2000       # 100m x 100m map (2000 x 2000 cells)
        self.map_height = 2000
        self.map_origin_x = -50.0   # Map origin in world coordinates
        self.map_origin_y = -50.0

        # SLAM parameters
        self.odom_topic = '/robot/odom'
        self.scan_topic = '/robot/laser_scan'
        self.map_publish_rate = 1.0  # Hz

        # Initialize map
        self.occupancy_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.occupancy_map.fill(-1)  # Unknown (-1), Free (0), Occupied (100)

        # Robot state
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.last_scan_time = rospy.Time.now()

        # TF listener
        self.tf_listener = TransformListener()

        # Publishers and subscribers
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        self.map_pub = rospy.Publisher('/robot/map', OccupancyGrid, queue_size=1)
        self.map_metadata_pub = rospy.Publisher('/robot/map_metadata', MapMetaData, queue_size=1)

        # Timer for map publishing
        self.map_publish_timer = rospy.Timer(
            rospy.Duration(1.0 / self.map_publish_rate),
            self.publish_map
        )

    def scan_callback(self, scan_msg):
        """Process LiDAR scan and update map"""
        # Get robot pose in map frame
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            rospy.logwarn("Could not get robot pose")
            return

        # Convert scan to world coordinates
        world_points = self.scan_to_world_coordinates(scan_msg, robot_pose)

        # Update occupancy grid
        self.update_occupancy_grid(world_points, robot_pose)

        # Update robot pose
        self.robot_pose = robot_pose

    def get_robot_pose(self):
        """Get robot pose from TF tree"""
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                '/map', '/base_link', rospy.Time(0)
            )
            # Convert quaternion to euler
            euler = tf.transformations.euler_from_quaternion(rot)
            return np.array([trans[0], trans[1], euler[2]])  # x, y, theta
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def scan_to_world_coordinates(self, scan_msg, robot_pose):
        """Convert LiDAR scan to world coordinates"""
        angles = np.linspace(
            scan_msg.angle_min,
            scan_msg.angle_max,
            len(scan_msg.ranges)
        )

        world_points = []
        for i, range_val in enumerate(scan_msg.ranges):
            if self.valid_range(range_val):
                # Calculate local coordinates
                local_x = range_val * np.cos(angles[i])
                local_y = range_val * np.sin(angles[i])

                # Transform to world coordinates
                cos_th = np.cos(robot_pose[2])
                sin_th = np.sin(robot_pose[2])

                world_x = robot_pose[0] + local_x * cos_th - local_y * sin_th
                world_y = robot_pose[1] + local_x * sin_th + local_y * cos_th

                world_points.append([world_x, world_y])

        return np.array(world_points)

    def valid_range(self, range_val):
        """Check if range value is valid"""
        return np.isfinite(range_val) and 0.1 < range_val < 30.0

    def update_occupancy_grid(self, world_points, robot_pose):
        """Update occupancy grid with new measurements"""
        if len(world_points) == 0:
            return

        # Calculate map coordinates for occupied cells
        map_xs = ((world_points[:, 0] - self.map_origin_x) / self.map_resolution).astype(int)
        map_ys = ((world_points[:, 1] - self.map_origin_y) / self.map_resolution).astype(int)

        # Validate coordinates are within map bounds
        valid_mask = (
            (0 <= map_xs) & (map_xs < self.map_width) &
            (0 <= map_ys) & (map_ys < self.map_height)
        )

        # Update occupied cells
        valid_xs = map_xs[valid_mask]
        valid_ys = map_ys[valid_mask]
        self.occupancy_map[valid_ys, valid_xs] = 100  # Occupied

        # Update free space along rays (ray tracing)
        self.update_free_space(robot_pose, world_points[valid_mask])

    def update_free_space(self, robot_pose, end_points):
        """Update free space along rays from robot to detected obstacles"""
        for end_point in end_points:
            # Calculate all points along the ray
            ray_points = self.ray_trace(robot_pose[:2], end_point)

            # Update free space
            for point in ray_points:
                map_x = int((point[0] - self.map_origin_x) / self.map_resolution)
                map_y = int((point[1] - self.map_origin_y) / self.map_resolution)

                if (0 <= map_x < self.map_width and 0 <= map_y < self.map_height):
                    if self.occupancy_map[map_y, map_x] == -1:  # Unknown
                        self.occupancy_map[map_y, map_x] = 0  # Free

    def ray_trace(self, start, end, step=0.05):
        """Trace a ray from start to end point"""
        # Calculate vector from start to end
        vec = end - start
        distance = np.linalg.norm(vec)

        if distance == 0:
            return []

        # Normalize direction vector
        direction = vec / distance

        # Generate points along the ray
        num_steps = int(distance / step)
        ray_points = []

        for i in range(num_steps):
            t = i * step / distance
            point = start + t * vec
            ray_points.append(point)

        return ray_points

    def publish_map(self, event):
        """Publish the occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = 'map'

        # Set map metadata
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatten occupancy map for message
        map_msg.data = self.occupancy_map.flatten().tolist()

        self.map_pub.publish(map_msg)

    def run(self):
        """Main SLAM loop"""
        rospy.loginfo("LiDAR SLAM started")
        rospy.spin()

if __name__ == '__main__':
    slam = LiDARSLAM()
    slam.run()
```

## LiDAR Simulation Validation

### Accuracy Assessment

```python
#!/usr/bin/env python3
"""
LiDAR simulation validation for digital twin systems
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64

class LiDARValidator:
    """Validates LiDAR simulation accuracy against ground truth"""

    def __init__(self):
        rospy.init_node('lidar_validator')

        # Configuration
        self.range_tolerance = 0.05  # 5cm tolerance
        self.angle_tolerance = 0.017 # 1 degree tolerance
        self.validation_window = 100  # Number of scans to average

        # Publishers for validation metrics
        self.range_accuracy_pub = rospy.Publisher('/lidar_validation/range_accuracy', Float64, queue_size=10)
        self.angle_accuracy_pub = rospy.Publisher('/lidar_validation/angle_accuracy', Float64, queue_size=10)
        self.overall_accuracy_pub = rospy.Publisher('/lidar_validation/overall_accuracy', Float64, queue_size=10)

        # Subscribers
        self.simulated_scan_sub = rospy.Subscriber('/robot/laser_scan', LaserScan, self.simulated_scan_callback)
        self.ground_truth_sub = rospy.Subscriber('/ground_truth/laser_scan', LaserScan, self.ground_truth_callback)

        # Validation buffers
        self.range_errors = []
        self.angle_errors = []
        self.validation_results = []

        # Ground truth storage
        self.ground_truth_scan = None
        self.last_comparison_time = rospy.Time.now()

    def simulated_scan_callback(self, scan_msg):
        """Process simulated LiDAR scan"""
        if self.ground_truth_scan is not None:
            # Compare with ground truth
            range_accuracy, angle_accuracy = self.compare_scans(scan_msg, self.ground_truth_scan)

            # Store results
            self.range_errors.append(range_accuracy)
            self.angle_errors.append(angle_accuracy)

            # Calculate overall accuracy
            overall_accuracy = (range_accuracy + angle_accuracy) / 2.0

            # Publish metrics
            self.range_accuracy_pub.publish(Float64(range_accuracy))
            self.angle_accuracy_pub.publish(Float64(angle_accuracy))
            self.overall_accuracy_pub.publish(Float64(overall_accuracy))

            # Log validation results periodically
            if len(self.range_errors) % 10 == 0:
                self.log_validation_results()

    def ground_truth_callback(self, gt_msg):
        """Store ground truth scan data"""
        self.ground_truth_scan = gt_msg

    def compare_scans(self, sim_scan, gt_scan):
        """Compare simulated scan with ground truth"""
        # Ensure both scans have the same parameters
        if (len(sim_scan.ranges) != len(gt_scan.ranges) or
            abs(sim_scan.angle_min - gt_scan.angle_min) > 0.001 or
            abs(sim_scan.angle_max - gt_scan.angle_max) > 0.001):
            rospy.logwarn("Scan parameters don't match for comparison")
            return 0.0, 0.0

        # Calculate range errors
        range_errors = []
        for i in range(len(sim_scan.ranges)):
            sim_range = sim_scan.ranges[i]
            gt_range = gt_scan.ranges[i]

            if np.isfinite(sim_range) and np.isfinite(gt_range):
                error = abs(sim_range - gt_range)
                range_errors.append(error)

        # Calculate angle errors (small angle differences)
        angle_errors = []
        for i in range(len(sim_scan.ranges)):
            # In this simplified model, angle accuracy is assumed perfect
            # In reality, this would involve more complex geometric comparisons
            angle_errors.append(0.0)

        # Calculate accuracy metrics (higher is better)
        if range_errors:
            avg_range_error = np.mean(range_errors)
            range_accuracy = max(0.0, 1.0 - avg_range_error / self.range_tolerance)
        else:
            range_accuracy = 1.0

        if angle_errors:
            avg_angle_error = np.mean(angle_errors)
            angle_accuracy = max(0.0, 1.0 - avg_angle_error / self.angle_tolerance)
        else:
            angle_accuracy = 1.0

        return range_accuracy, angle_accuracy

    def log_validation_results(self):
        """Log validation statistics"""
        if not self.range_errors:
            return

        # Calculate statistics
        recent_errors = self.range_errors[-10:]  # Last 10 measurements
        avg_range_error = np.mean(recent_errors)
        std_range_error = np.std(recent_errors)

        # Calculate accuracy percentage
        accuracy_percentage = (sum(1 for e in recent_errors if e <= self.range_tolerance) / len(recent_errors)) * 100

        rospy.loginfo(
            f"LiDAR Validation - "
            f"Accuracy: {accuracy_percentage:.1f}% "
            f"Avg Error: {avg_range_error:.3f}m "
            f"Std Dev: {std_range_error:.3f}m"
        )

        # Check for performance degradation
        if accuracy_percentage < 90:  # Less than 90% accuracy
            rospy.logwarn(f"LiDAR simulation accuracy degraded to {accuracy_percentage:.1f}%")

    def generate_validation_report(self):
        """Generate comprehensive validation report"""
        if not self.range_errors:
            rospy.loginfo("No validation data available")
            return

        # Overall statistics
        avg_range_error = np.mean(self.range_errors)
        max_range_error = np.max(self.range_errors)
        min_range_error = np.min(self.range_errors)
        std_range_error = np.std(self.range_errors)

        total_measurements = len(self.range_errors)
        valid_measurements = sum(1 for e in self.range_errors if e <= self.range_tolerance)
        accuracy_percentage = (valid_measurements / total_measurements) * 100

        rospy.loginfo("\n=== LiDAR Simulation Validation Report ===")
        rospy.loginfo(f"Total measurements: {total_measurements}")
        rospy.loginfo(f"Valid measurements: {valid_measurements} ({accuracy_percentage:.1f}%)")
        rospy.loginfo(f"Average range error: {avg_range_error:.3f} m")
        rospy.loginfo(f"Maximum range error: {max_range_error:.3f} m")
        rospy.loginfo(f"Minimum range error: {min_range_error:.3f} m")
        rospy.loginfo(f"Standard deviation: {std_range_error:.3f} m")
        rospy.loginfo(f"Range tolerance: {self.range_tolerance} m")
        rospy.loginfo("===========================================")

        # Return validation summary
        return {
            'accuracy_percentage': accuracy_percentage,
            'average_error': avg_range_error,
            'max_error': max_range_error,
            'std_deviation': std_range_error,
            'is_valid': accuracy_percentage >= 90
        }

    def run(self):
        """Main validation loop"""
        rate = rospy.Rate(1)  # Log once per second

        while not rospy.is_shutdown():
            rate.sleep()

        # Generate final report on shutdown
        self.generate_validation_report()

if __name__ == '__main__':
    validator = LiDARValidator()
    rospy.loginfo("LiDAR validator started")
    validator.run()
```

## Summary

LiDAR simulation is a critical component of digital twin systems for humanoid robots. Proper configuration of LiDAR sensors with realistic parameters, noise models, and validation ensures that the digital twin accurately represents the physical robot's perception capabilities. The implementation of processing pipelines, SLAM algorithms, and validation systems enables effective training and testing of perception and navigation systems before deployment on physical robots.

## Next Steps

Continue to learn about depth camera simulation and its implementation in digital twin environments.

[← Previous: Sensor Simulation](./sensor-simulation) | [Next: Depth Camera Simulation →](./depth-camera-simulation)