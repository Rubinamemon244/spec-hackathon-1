---
sidebar_position: 20
---

# Depth Camera Simulation for Humanoid Robots

## Learning Objectives

By the end of this section, you will be able to:
- Configure realistic depth cameras in simulation environments
- Understand depth camera principles and point cloud generation
- Implement depth camera noise models matching real-world characteristics
- Process depth camera data for computer vision applications
- Validate depth camera simulation accuracy for digital twin applications

## Overview

Depth cameras are essential sensors for humanoid robots, providing 3D spatial information that enables computer vision applications such as object recognition, scene understanding, and manipulation. In digital twin environments, realistic depth camera simulation is crucial for training perception algorithms and validating vision-based systems before deployment on physical robots. This section covers the principles, configuration, and implementation of depth camera simulation.

## Depth Camera Fundamentals

### How Depth Cameras Work

Depth cameras capture both color (RGB) and depth information simultaneously:

- **RGB Information**: Color data for visual recognition and appearance
- **Depth Information**: Distance data for 3D reconstruction and spatial understanding
- **IR Illumination**: Many depth cameras use infrared light to measure depth
- **Stereo Vision**: Some cameras use stereo triangulation for depth measurement

### Depth Camera Specifications

Key parameters that affect depth camera performance:

- **Resolution**: Image resolution (e.g., 640×480, 1280×720)
- **Frame Rate**: Acquisition speed (typically 30-60 Hz)
- **Range**: Operating distance (e.g., 0.3m to 10m)
- **Accuracy**: Depth measurement precision (±mm to ±cm)
- **Field of View**: Angular coverage (diagonal, horizontal, vertical)
- **Noise Characteristics**: Random and systematic errors in measurements

## Depth Camera Configuration in Gazebo

### Basic Depth Camera Configuration

```xml
<!-- Example: Basic depth camera configuration -->
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees in radians -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>      <!-- Near clipping plane (meters) -->
      <far>10.0</far>       <!-- Far clipping plane (meters) -->
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>  <!-- 7mm standard deviation -->
    </noise>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>  <!-- 30 Hz update rate -->
  <visualize>true</visualize>     <!-- Visualize in GUI -->

  <!-- Plugin configuration -->
  <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <baseline>0.2</baseline>
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <cameraName>camera</cameraName>
    <imageTopicName>/camera/rgb/image_raw</imageTopicName>
    <depthImageTopicName>/camera/depth/image_raw</depthImageTopicName>
    <pointCloudTopicName>/camera/depth/points</pointCloudTopicName>
    <cameraInfoTopicName>/camera/rgb/camera_info</cameraInfoTopicName>
    <depthImageCameraInfoTopicName>/camera/depth/camera_info</depthImageCameraInfoTopicName>
    <frameName>camera_depth_optical_frame</frameName>
    <pointCloudCutoff>0.1</pointCloudCutoff>
    <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
    <CxPrime>0</CxPrime>
    <Cx>320.5</Cx>
    <Cy>240.5</Cy>
    <focalLength>320</focalLength>
    <hackBaseline>0</hackBaseline>
  </plugin>
</sensor>
```

### Advanced Depth Camera Configuration

```xml
<!-- Example: Advanced depth camera with detailed parameters -->
<sensor name="advanced_depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
    <image>
      <width>1280</width>     <!-- High resolution -->
      <height>720</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.05</near>      <!-- Close range capability -->
      <far>15.0</far>        <!-- Extended range -->
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.005</stddev>  <!-- Improved accuracy -->
    </noise>
  </camera>
  <always_on>1</always_on>
  <update_rate>60</update_rate>  <!-- High frame rate -->
  <visualize>true</visualize>

  <!-- Advanced plugin configuration -->
  <plugin name="kinect_camera_controller" filename="libgazebo_ros_openni_kinect.so">
    <baseline>0.1</baseline>
    <alwaysOn>true</alwaysOn>
    <updateRate>60.0</updateRate>
    <cameraName>kinect_camera</cameraName>

    <!-- Topic names -->
    <imageTopicName>/camera/rgb/image_color</imageTopicName>
    <depthImageTopicName>/camera/depth_registered/image_raw</depthImageTopicName>
    <pointCloudTopicName>/camera/depth_registered/points</pointCloudTopicName>
    <cameraInfoTopicName>/camera/rgb/camera_info</cameraInfoTopicName>
    <depthImageCameraInfoTopicName>/camera/depth_registered/camera_info</depthImageCameraInfoTopicName>

    <!-- Frame names -->
    <frameName>camera_rgb_optical_frame</frameName>
    <depthImageFrameName>camera_depth_optical_frame</depthImageFrameName>

    <!-- Point cloud parameters -->
    <pointCloudCutoff>0.05</pointCloudCutoff>      <!-- Near cutoff -->
    <pointCloudCutoffMax>12.0</pointCloudCutoffMax> <!-- Far cutoff -->

    <!-- Camera intrinsics -->
    <CxPrime>0</CxPrime>
    <Cx>640.5</Cx>    <!-- Principal point x -->
    <Cy>360.5</Cy>    <!-- Principal point y -->
    <focalLength>640</focalLength>  <!-- Focal length in pixels -->

    <!-- Distortion parameters -->
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>

    <!-- Hack parameters -->
    <hackBaseline>0</hackBaseline>
    <disableDistortion>0</disableDistortion>
  </plugin>
</sensor>
```

## Depth Camera Data Processing

### Depth Image Processing Pipeline

```python
#!/usr/bin/env python3
"""
Depth camera processing pipeline for humanoid robot digital twin
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import struct

class DepthCameraProcessor:
    """Processes depth camera data for humanoid robot applications"""

    def __init__(self):
        rospy.init_node('depth_camera_processor')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Publishers
        self.processed_depth_pub = rospy.Publisher('/camera/depth/processed', Image, queue_size=10)
        self.filtered_pointcloud_pub = rospy.Publisher('/camera/filtered_points', PointCloud2, queue_size=10)
        self.object_mask_pub = rospy.Publisher('/camera/object_masks', Image, queue_size=10)

        # Subscribers
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        self.info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.camera_info_callback)

        # Camera parameters (will be updated from camera info)
        self.fx = 525.0  # Focal length x
        self.fy = 525.0  # Focal length y
        self.cx = 319.5  # Principal point x
        self.cy = 239.5  # Principal point y

        # Processing parameters
        self.min_depth = 0.1    # Minimum depth (meters)
        self.max_depth = 10.0   # Maximum depth (meters)
        self.depth_noise_threshold = 0.05  # Noise threshold (meters)
        self.surface_normal_threshold = 0.1  # Surface normal threshold

        # Storage
        self.rgb_image = None
        self.camera_info = None

    def camera_info_callback(self, info_msg):
        """Update camera parameters from camera info"""
        self.fx = info_msg.K[0]  # K[0] is fx
        self.fy = info_msg.K[4]  # K[4] is fy
        self.cx = info_msg.K[2]  # K[2] is cx
        self.cy = info_msg.K[5]  # K[5] is cy

    def depth_callback(self, depth_msg):
        """Process incoming depth image"""
        try:
            # Convert ROS Image message to OpenCV image
            depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

            # Validate depth image
            depth_cv = self.validate_depth_image(depth_cv)

            # Apply filtering
            filtered_depth = self.filter_depth_image(depth_cv)

            # Generate point cloud
            pointcloud = self.depth_to_pointcloud(filtered_depth, depth_msg.header)

            # Detect objects in depth
            object_mask = self.detect_objects_in_depth(filtered_depth)

            # Publish processed data
            self.publish_processed_depth(filtered_depth, depth_msg.header)
            self.publish_pointcloud(pointcloud, depth_msg.header)
            self.publish_object_mask(object_mask, depth_msg.header)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

    def rgb_callback(self, rgb_msg):
        """Process incoming RGB image"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error in RGB callback: {e}")

    def validate_depth_image(self, depth_image):
        """Validate and clean depth image"""
        # Replace invalid depth values (NaN, infinity) with max depth
        depth_image = np.nan_to_num(depth_image, nan=self.max_depth, posinf=self.max_depth, neginf=0)

        # Clamp depth values to valid range
        depth_image = np.clip(depth_image, self.min_depth, self.max_depth)

        return depth_image

    def filter_depth_image(self, depth_image):
        """Apply noise filtering to depth image"""
        # Apply bilateral filter to preserve edges while reducing noise
        filtered = cv2.bilateralFilter(depth_image, 5, 50, 50)

        # Apply median filter to remove salt-and-pepper noise
        filtered = cv2.medianBlur(filtered.astype(np.float32), 3)

        # Apply validity mask based on depth range
        valid_mask = (depth_image >= self.min_depth) & (depth_image <= self.max_depth)
        filtered = np.where(valid_mask, filtered, self.max_depth)

        return filtered

    def depth_to_pointcloud(self, depth_image, header):
        """Convert depth image to point cloud"""
        height, width = depth_image.shape

        # Generate pixel coordinates
        u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))

        # Convert to normalized image coordinates
        x_norm = (u_coords - self.cx) / self.fx
        y_norm = (v_coords - self.cy) / self.fy

        # Calculate 3D coordinates
        z = depth_image  # Depth values
        x = x_norm * z
        y = y_norm * z

        # Create point cloud
        points = np.stack([x, y, z], axis=-1).reshape(-1, 3)

        # Filter out invalid points
        valid_points = points[np.isfinite(points).all(axis=1)]
        valid_points = valid_points[valid_points[:, 2] > 0]  # Only positive depths

        return valid_points

    def detect_objects_in_depth(self, depth_image):
        """Detect objects using depth discontinuities"""
        # Calculate depth gradients
        grad_x = cv2.Sobel(depth_image, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(depth_image, cv2.CV_32F, 0, 1, ksize=3)
        gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)

        # Threshold to detect depth discontinuities (object boundaries)
        object_mask = (gradient_magnitude > self.depth_noise_threshold).astype(np.uint8) * 255

        # Apply morphological operations to clean up the mask
        kernel = np.ones((3, 3), np.uint8)
        object_mask = cv2.morphologyEx(object_mask, cv2.MORPH_CLOSE, kernel)
        object_mask = cv2.morphologyEx(object_mask, cv2.MORPH_OPEN, kernel)

        return object_mask

    def publish_processed_depth(self, depth_image, header):
        """Publish processed depth image"""
        try:
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image.astype(np.float32), encoding='32FC1')
            depth_msg.header = header
            self.processed_depth_pub.publish(depth_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error in publish_processed_depth: {e}")

    def publish_pointcloud(self, points, header):
        """Publish point cloud"""
        if len(points) == 0:
            return

        # Create header
        cloud_header = Header()
        cloud_header.stamp = header.stamp
        cloud_header.frame_id = header.frame_id

        # Define point fields
        fields = [
            point_cloud2.PointField('x', 0, point_cloud2.PointField.FLOAT32, 1),
            point_cloud2.PointField('y', 4, point_cloud2.PointField.FLOAT32, 1),
            point_cloud2.PointField('z', 8, point_cloud2.PointField.FLOAT32, 1)
        ]

        # Create and publish point cloud
        cloud_msg = point_cloud2.create_cloud(cloud_header, fields, points)
        self.filtered_pointcloud_pub.publish(cloud_msg)

    def publish_object_mask(self, mask, header):
        """Publish object mask"""
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = header
            self.object_mask_pub.publish(mask_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error in publish_object_mask: {e}")

    def run(self):
        """Main processing loop"""
        rospy.loginfo("Depth camera processor started")
        rospy.spin()

if __name__ == '__main__':
    processor = DepthCameraProcessor()
    processor.run()
```

## 3D Reconstruction from Depth

### Point Cloud Processing and Mesh Generation

```python
#!/usr/bin/env python3
"""
3D reconstruction from depth camera data
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import open3d as o3d
import cv2

class DepthReconstructor:
    """Reconstructs 3D models from depth camera data"""

    def __init__(self):
        rospy.init_node('depth_reconstructor')

        # Subscribers
        self.pointcloud_sub = rospy.Subscriber('/camera/depth_registered/points', PointCloud2, self.pointcloud_callback)

        # Publishers
        self.reconstruction_pub = rospy.Publisher('/camera/reconstructed_mesh', MarkerArray, queue_size=10)
        self.surface_normals_pub = rospy.Publisher('/camera/surface_normals', MarkerArray, queue_size=10)

        # Processing parameters
        self.voxel_size = 0.02  # 2cm voxel size
        self.max_range = 5.0    # Maximum reconstruction range
        self.min_points_per_cluster = 10  # Minimum points for object cluster

        # Reconstruction state
        self.accumulated_points = []
        self.accumulation_window = 10  # Accumulate 10 frames

    def pointcloud_callback(self, cloud_msg):
        """Process incoming point cloud data"""
        try:
            # Convert PointCloud2 to numpy array
            points_list = []
            for point in point_cloud2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
                if all(np.isfinite(coord) for coord in point):
                    if np.linalg.norm(point) <= self.max_range:  # Range filter
                        points_list.append([point[0], point[1], point[2]])

            if not points_list:
                return

            # Add points to accumulation buffer
            self.accumulated_points.extend(points_list)

            # Process accumulated points if we have enough
            if len(self.accumulated_points) >= self.accumulation_window * 100:  # Arbitrary threshold
                self.reconstruct_scene()
                self.accumulated_points = []  # Reset accumulator

        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")

    def reconstruct_scene(self):
        """Perform 3D reconstruction from accumulated points"""
        if len(self.accumulated_points) < 100:  # Need minimum points
            return

        # Convert to Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(self.accumulated_points))

        # Downsample point cloud
        pcd_downsampled = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        # Estimate surface normals
        pcd_downsampled.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )

        # Statistical outlier removal
        pcd_filtered, _ = pcd_downsampled.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        # Perform surface reconstruction using Poisson
        try:
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                pcd_filtered, depth=8
            )

            # Filter mesh based on density (remove low-density triangles)
            vertices_to_remove = densities < np.quantile(densities, 0.01)
            mesh.remove_vertices_by_mask(vertices_to_remove)

            # Publish reconstructed mesh
            self.publish_mesh(mesh)

            # Publish surface normals visualization
            self.publish_surface_normals(pcd_filtered)

        except Exception as e:
            rospy.logwarn(f"Poisson reconstruction failed: {e}")
            # Fallback to simple point cloud visualization
            self.publish_point_cloud_visualization(pcd_filtered)

    def publish_mesh(self, mesh):
        """Publish reconstructed mesh as visualization markers"""
        marker_array = MarkerArray()

        # Create marker for the mesh
        mesh_marker = Marker()
        mesh_marker.header = Header()
        mesh_marker.header.stamp = rospy.Time.now()
        mesh_marker.header.frame_id = "camera_rgb_optical_frame"
        mesh_marker.ns = "reconstructed_mesh"
        mesh_marker.id = 0
        mesh_marker.type = Marker.TRIANGLE_LIST
        mesh_marker.action = Marker.ADD

        # Set mesh scale
        mesh_marker.scale.x = 1.0
        mesh_marker.scale.y = 1.0
        mesh_marker.scale.z = 1.0

        # Set mesh color (light blue)
        mesh_marker.color.r = 0.5
        mesh_marker.color.g = 0.8
        mesh_marker.color.b = 1.0
        mesh_marker.color.a = 0.7  # Semi-transparent

        # Extract triangle vertices
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)

        for triangle in triangles:
            for vertex_idx in triangle:
                point = Point()
                point.x = vertices[vertex_idx][0]
                point.y = vertices[vertex_idx][1]
                point.z = vertices[vertex_idx][2]
                mesh_marker.points.append(point)

        marker_array.markers.append(mesh_marker)
        self.reconstruction_pub.publish(marker_array)

    def publish_surface_normals(self, pcd):
        """Publish surface normals as visualization markers"""
        marker_array = MarkerArray()

        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)

        for i in range(0, len(points), 10):  # Show every 10th normal for clarity
            normal_marker = Marker()
            normal_marker.header = Header()
            normal_marker.header.stamp = rospy.Time.now()
            normal_marker.header.frame_id = "camera_rgb_optical_frame"
            normal_marker.ns = "surface_normals"
            normal_marker.id = i
            normal_marker.type = Marker.ARROW
            normal_marker.action = Marker.ADD

            # Set arrow scale
            normal_marker.scale.x = 0.05  # Shaft length
            normal_marker.scale.y = 0.01  # Head width
            normal_marker.scale.z = 0.01  # Head length

            # Set color (green for normals)
            normal_marker.color.r = 0.0
            normal_marker.color.g = 1.0
            normal_marker.color.b = 0.0
            normal_marker.color.a = 1.0

            # Set start and end points
            start_point = Point()
            start_point.x = points[i][0]
            start_point.y = points[i][1]
            start_point.z = points[i][2]

            end_point = Point()
            normal_scale = 0.1  # Scale the normal vector
            end_point.x = points[i][0] + normals[i][0] * normal_scale
            end_point.y = points[i][1] + normals[i][1] * normal_scale
            end_point.z = points[i][2] + normals[i][2] * normal_scale

            normal_marker.points.append(start_point)
            normal_marker.points.append(end_point)

            marker_array.markers.append(normal_marker)

        self.surface_normals_pub.publish(marker_array)

    def publish_point_cloud_visualization(self, pcd):
        """Publish point cloud as visualization markers (fallback)"""
        marker_array = MarkerArray()

        points = np.asarray(pcd.points)

        # Create point cloud marker
        pc_marker = Marker()
        pc_marker.header = Header()
        pc_marker.header.stamp = rospy.Time.now()
        pc_marker.header.frame_id = "camera_rgb_optical_frame"
        pc_marker.ns = "point_cloud"
        pc_marker.id = 0
        pc_marker.type = Marker.POINTS
        pc_marker.action = Marker.ADD

        # Set point scale
        pc_marker.scale.x = 0.01
        pc_marker.scale.y = 0.01
        pc_marker.scale.z = 0.01

        # Set color
        pc_marker.color.r = 1.0
        pc_marker.color.g = 1.0
        pc_marker.color.b = 1.0
        pc_marker.color.a = 1.0

        # Add points
        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            pc_marker.points.append(p)

        marker_array.markers.append(pc_marker)
        self.reconstruction_pub.publish(marker_array)

    def run(self):
        """Main reconstruction loop"""
        rospy.loginfo("Depth reconstructor started")
        rospy.spin()

if __name__ == '__main__':
    reconstructor = DepthReconstructor()
    reconstructor.run()
```

## Depth Camera SLAM

### Visual-Inertial SLAM with Depth Information

```python
#!/usr/bin/env python3
"""
Depth camera SLAM for humanoid robot digital twin
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf
import cv2
from scipy.spatial.transform import Rotation as R

class DepthSLAM:
    """Depth camera SLAM implementation"""

    def __init__(self):
        rospy.init_node('depth_slam')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Subscribers
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_callback)
        self.info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.camera_info_callback)

        # Publishers
        self.pose_pub = rospy.Publisher('/camera/pose', PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/camera/odometry', Odometry, queue_size=10)

        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Camera parameters
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5

        # SLAM state
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.previous_features = None
        self.previous_depth = None
        self.keyframes = []
        self.map_points = []

        # Processing parameters
        self.feature_detector = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.min_matches = 20
        self.max_translation = 1.0  # Maximum expected translation between frames

        # Timing
        self.last_process_time = rospy.Time.now()

    def camera_info_callback(self, info_msg):
        """Update camera parameters from camera info"""
        self.fx = info_msg.K[0]
        self.fy = info_msg.K[4]
        self.cx = info_msg.K[2]
        self.cy = info_msg.K[5]

    def rgb_callback(self, rgb_msg):
        """Process RGB image for feature tracking"""
        try:
            # Convert to grayscale
            rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(rgb_cv, cv2.COLOR_BGR2GRAY)

            # Extract features
            keypoints = self.feature_detector.detect(gray)
            keypoints, descriptors = self.feature_detector.compute(gray, keypoints)

            if descriptors is not None and len(descriptors) > 0:
                # Store current features
                current_features = {
                    'keypoints': keypoints,
                    'descriptors': descriptors,
                    'image': gray,
                    'timestamp': rgb_msg.header.stamp
                }

                # Estimate pose if we have previous features
                if self.previous_features is not None:
                    transform = self.estimate_transform(
                        self.previous_features, current_features, self.previous_depth
                    )

                    if transform is not None:
                        # Update pose
                        self.current_pose = self.current_pose @ transform

                        # Publish pose and odometry
                        self.publish_pose(rgb_msg.header)
                        self.publish_odometry(rgb_msg.header)

                        # Broadcast TF
                        self.broadcast_transform(rgb_msg.header)

                # Update previous features
                self.previous_features = current_features

        except Exception as e:
            rospy.logerr(f"Error in RGB callback: {e}")

    def depth_callback(self, depth_msg):
        """Process depth image"""
        try:
            # Convert depth image
            depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')

            # Validate depth image
            depth_cv = np.nan_to_num(depth_cv, nan=0, posinf=0, neginf=0)
            depth_cv = np.clip(depth_cv, 0, 10.0)  # Clamp to reasonable range

            # Store depth image
            self.previous_depth = depth_cv

        except Exception as e:
            rospy.logerr(f"Error in depth callback: {e}")

    def estimate_transform(self, prev_features, curr_features, depth_image):
        """Estimate 3D transform between consecutive frames"""
        if depth_image is None:
            return None

        # Match features
        matches = self.matcher.match(
            prev_features['descriptors'],
            curr_features['descriptors']
        )

        # Sort matches by distance
        matches = sorted(matches, key=lambda x: x.distance)

        # Keep only good matches
        good_matches = [m for m in matches if m.distance < 50]  # Threshold

        if len(good_matches) < self.min_matches:
            rospy.logwarn(f"Not enough good matches: {len(good_matches)} < {self.min_matches}")
            return None

        # Get corresponding points
        prev_pts = np.float32([prev_features['keypoints'][m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        curr_pts = np.float32([curr_features['keypoints'][m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # Get 3D points from previous depth image
        prev_3d_points = []
        for pt in prev_pts.reshape(-1, 2):
            u, v = int(pt[0]), int(pt[1])
            if 0 <= u < depth_image.shape[1] and 0 <= v < depth_image.shape[0]:
                depth = depth_image[v, u]
                if depth > 0:  # Valid depth
                    # Convert pixel to 3D point
                    x = (u - self.cx) * depth / self.fx
                    y = (v - self.cy) * depth / self.fy
                    z = depth
                    prev_3d_points.append([x, y, z])

        if len(prev_3d_points) < self.min_matches:
            rospy.logwarn(f"Not enough valid 3D points: {len(prev_3d_points)} < {self.min_matches}")
            return None

        prev_3d_points = np.array(prev_3d_points)
        curr_2d_points = curr_pts.reshape(-1, 2)

        # Solve Perspective-n-Point problem
        try:
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                prev_3d_points,
                curr_2d_points,
                np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]]),
                distCoeffs=None,
                reprojectionError=5.0,
                iterationsCount=100
            )

            if success and tvec is not None:
                # Convert rotation vector to rotation matrix
                R_mat, _ = cv2.Rodrigues(rvec)

                # Create transformation matrix
                transform = np.eye(4)
                transform[:3, :3] = R_mat
                transform[:3, 3] = tvec.flatten()

                # Check for reasonable motion
                translation_norm = np.linalg.norm(tvec)
                if translation_norm > self.max_translation:
                    rospy.logwarn(f"Excessive translation detected: {translation_norm:.3f}m")
                    return None

                return transform

        except Exception as e:
            rospy.logwarn(f"PnP solution failed: {e}")

        return None

    def publish_pose(self, header):
        """Publish current camera pose"""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = "map"

        # Extract position and orientation from transformation matrix
        position = self.current_pose[:3, 3]
        rotation_matrix = self.current_pose[:3, :3]

        # Convert rotation matrix to quaternion
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()  # [x, y, z, w]

        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.pose_pub.publish(pose_msg)

    def publish_odometry(self, header):
        """Publish odometry information"""
        odom_msg = Odometry()
        odom_msg.header = header
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "camera_depth_optical_frame"

        # Set pose
        position = self.current_pose[:3, 3]
        rotation_matrix = self.current_pose[:3, :3]
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()

        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]

        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Set zero velocity (would need temporal differencing for real velocity)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

    def broadcast_transform(self, header):
        """Broadcast camera transform to TF tree"""
        position = self.current_pose[:3, 3]
        rotation_matrix = self.current_pose[:3, :3]
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()

        self.tf_broadcaster.sendTransform(
            (position[0], position[1], position[2]),
            (quat[0], quat[1], quat[2], quat[3]),
            header.stamp,
            "camera_depth_optical_frame",
            "map"
        )

    def run(self):
        """Main SLAM loop"""
        rospy.loginfo("Depth SLAM started")
        rospy.spin()

if __name__ == '__main__':
    slam = DepthSLAM()
    slam.run()
```

## Depth Camera Validation

### Accuracy Assessment and Calibration

```python
#!/usr/bin/env python3
"""
Depth camera validation for digital twin systems
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2

class DepthValidator:
    """Validates depth camera simulation accuracy"""

    def __init__(self):
        rospy.init_node('depth_validator')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Publishers for validation metrics
        self.accuracy_pub = rospy.Publisher('/depth_validation/accuracy', Float64, queue_size=10)
        self.precision_pub = rospy.Publisher('/depth_validation/precision', Float64, queue_size=10)
        self.range_accuracy_pub = rospy.Publisher('/depth_validation/range_accuracy', Float64, queue_size=10)

        # Subscribers
        self.simulated_depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.simulated_depth_callback)
        self.ground_truth_depth_sub = rospy.Subscriber('/ground_truth/depth/image_raw', Image, self.ground_truth_depth_callback)

        # Validation parameters
        self.range_tolerance = 0.05  # 5cm tolerance
        self.validation_window = 50  # Number of frames to average
        self.min_depth = 0.1
        self.max_depth = 10.0

        # Storage for validation
        self.simulated_depth = None
        self.ground_truth_depth = None
        self.error_history = []

    def simulated_depth_callback(self, sim_msg):
        """Process simulated depth image"""
        try:
            # Convert to numpy array
            sim_depth = self.bridge.imgmsg_to_cv2(sim_msg, desired_encoding='32FC1')
            self.simulated_depth = sim_depth

            # If we have ground truth, validate
            if self.ground_truth_depth is not None:
                self.validate_depth_accuracy()

        except Exception as e:
            rospy.logerr(f"Error processing simulated depth: {e}")

    def ground_truth_depth_callback(self, gt_msg):
        """Process ground truth depth image"""
        try:
            gt_depth = self.bridge.imgmsg_to_cv2(gt_msg, desired_encoding='32FC1')
            self.ground_truth_depth = gt_depth

        except Exception as e:
            rospy.logerr(f"Error processing ground truth depth: {e}")

    def validate_depth_accuracy(self):
        """Validate simulated depth against ground truth"""
        if self.simulated_depth is None or self.ground_truth_depth is None:
            return

        # Ensure images are the same size
        if self.simulated_depth.shape != self.ground_truth_depth.shape:
            rospy.logwarn("Depth image sizes don't match")
            return

        # Calculate depth errors
        valid_mask = (
            (self.ground_truth_depth >= self.min_depth) &
            (self.ground_truth_depth <= self.max_depth) &
            np.isfinite(self.simulated_depth) &
            np.isfinite(self.ground_truth_depth)
        )

        if np.sum(valid_mask) == 0:
            rospy.logwarn("No valid depth pixels for validation")
            return

        # Calculate errors
        depth_errors = np.abs(self.simulated_depth - self.ground_truth_depth)
        valid_errors = depth_errors[valid_mask]

        if len(valid_errors) == 0:
            return

        # Calculate accuracy metrics
        rmse = np.sqrt(np.mean(valid_errors**2))  # Root Mean Square Error
        mae = np.mean(valid_errors)  # Mean Absolute Error
        std_error = np.std(valid_errors)  # Standard deviation of errors

        # Calculate percentage of measurements within tolerance
        within_tolerance = np.sum(valid_errors <= self.range_tolerance) / len(valid_errors)

        # Store in history
        self.error_history.append({
            'rmse': rmse,
            'mae': mae,
            'std_error': std_error,
            'within_tolerance': within_tolerance,
            'timestamp': rospy.Time.now()
        })

        # Keep only recent history
        if len(self.error_history) > self.validation_window:
            self.error_history.pop(0)

        # Calculate rolling averages
        if len(self.error_history) > 0:
            avg_rmse = np.mean([h['rmse'] for h in self.error_history])
            avg_mae = np.mean([h['mae'] for h in self.error_history])
            avg_within_tol = np.mean([h['within_tolerance'] for h in self.error_history])

            # Publish metrics
            self.accuracy_pub.publish(Float64(avg_mae))
            self.precision_pub.publish(Float64(avg_rmse))
            self.range_accuracy_pub.publish(Float64(avg_within_tol))

            # Log validation results
            if len(self.error_history) % 10 == 0:  # Log every 10 frames
                rospy.loginfo(
                    f"Depth Validation - MAE: {avg_mae:.3f}m, "
                    f"RMSE: {avg_rmse:.3f}m, "
                    f"Accuracy: {avg_within_tol:.1%}"
                )

                # Check for validation failure
                if avg_within_tol < 0.8:  # Less than 80% accuracy
                    rospy.logwarn(f"Depth camera accuracy degraded: {avg_within_tol:.1%}")

    def generate_validation_report(self):
        """Generate comprehensive validation report"""
        if not self.error_history:
            rospy.loginfo("No validation data available")
            return

        # Calculate statistics
        rmses = [h['rmse'] for h in self.error_history]
        maes = [h['mae'] for h in self.error_history]
        accuracies = [h['within_tolerance'] for h in self.error_history]

        avg_rmse = np.mean(rmses)
        avg_mae = np.mean(maes)
        avg_accuracy = np.mean(accuracies)
        std_rmse = np.std(rmses)
        std_mae = np.std(maes)

        rospy.loginfo("\n=== Depth Camera Validation Report ===")
        rospy.loginfo(f"Average RMSE: {avg_rmse:.3f} ± {std_rmse:.3f} m")
        rospy.loginfo(f"Average MAE: {avg_mae:.3f} ± {std_mae:.3f} m")
        rospy.loginfo(f"Average accuracy within {self.range_tolerance}m: {avg_accuracy:.1%}")
        rospy.loginfo(f"Validation window: {len(self.error_history)} frames")
        rospy.loginfo("=====================================")

        # Return validation summary
        return {
            'rmse': avg_rmse,
            'mae': avg_mae,
            'accuracy': avg_accuracy,
            'is_valid': avg_accuracy >= 0.8  # 80% threshold
        }

    def run(self):
        """Main validation loop"""
        rate = rospy.Rate(1)  # Log once per second

        while not rospy.is_shutdown():
            rate.sleep()

        # Generate final report on shutdown
        self.generate_validation_report()

if __name__ == '__main__':
    validator = DepthValidator()
    rospy.loginfo("Depth validator started")
    validator.run()
```

## Summary

Depth camera simulation is crucial for creating realistic digital twins of humanoid robots. Proper configuration with realistic parameters, noise models, and validation ensures that the digital twin accurately represents the physical robot's perception capabilities. The implementation of processing pipelines, 3D reconstruction algorithms, and SLAM systems enables effective training and testing of vision-based systems before deployment on physical robots.

## Next Steps

Continue to learn about IMU simulation and its implementation in digital twin environments.

[← Previous: LiDAR Simulation](./lidar-simulation) | [Next: IMU Simulation →](./imu-simulation)