---
sidebar_position: 18
---

# Sensor Simulation for Humanoid Robots

## Learning Objectives

By the end of this section, you will be able to:
- Understand the different types of sensors used in humanoid robots
- Implement realistic sensor simulation for LiDAR, depth cameras, and IMUs
- Configure sensor parameters to match real-world characteristics
- Validate sensor simulation accuracy for digital twin applications

## Overview

Sensor simulation is a critical component of digital twin systems for humanoid robots. Realistic sensor data allows AI systems to be trained and tested in simulation before deployment on physical robots. This section covers the main sensor types used in humanoid robotics and how to simulate them accurately in digital twin environments.

## Types of Sensors in Humanoid Robots

### LiDAR Sensors

LiDAR (Light Detection and Ranging) sensors are crucial for humanoid robots:

- **Function**: Generate 3D point clouds of the environment
- **Applications**: Mapping, navigation, obstacle detection
- **Simulation Requirements**: Accurate geometric modeling, realistic noise patterns
- **Digital Twin Role**: Provides spatial awareness in the virtual environment

### Depth Cameras

Depth cameras provide 3D perception capabilities:

- **Function**: Capture depth information for each pixel
- **Applications**: Object recognition, scene understanding, manipulation
- **Simulation Requirements**: Realistic depth noise, field of view accuracy
- **Digital Twin Role**: Enables computer vision algorithm training

### IMU Sensors

Inertial Measurement Units provide motion and orientation data:

- **Function**: Measure acceleration, angular velocity, and orientation
- **Applications**: Balance control, motion tracking, navigation
- **Simulation Requirements**: Accurate noise modeling, drift simulation
- **Digital Twin Role**: Provides internal state awareness for the digital twin

## LiDAR Simulation

### LiDAR Configuration in Gazebo

```xml
<!-- Example LiDAR sensor configuration in Gazebo -->
<sensor name="laser_front" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>        <!-- Number of rays per revolution -->
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
  <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
    <topicName>/robot/laser_scan</topicName>
    <frameName>laser_frame</frameName>
  </plugin>
</sensor>
```

### LiDAR Sensor Model Implementation

```python
#!/usr/bin/env python3
"""
LiDAR sensor simulation for humanoid robot digital twin
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
from std_msgs.msg import Header

class LiDARSensorSimulator:
    """Simulates LiDAR sensor for humanoid robot digital twin"""

    def __init__(self):
        rospy.init_node('lidar_simulator')

        # LiDAR configuration parameters
        self.angle_min = -np.pi/2      # -90 degrees
        self.angle_max = np.pi/2       # 90 degrees
        self.angle_increment = np.pi / 180  # 1 degree resolution
        self.scan_time = 0.1           # 10 Hz
        self.range_min = 0.1           # 10 cm minimum
        self.range_max = 30.0          # 30 m maximum
        self.noise_std_dev = 0.02      # 2 cm noise standard deviation

        # Number of rays
        self.num_rays = int((self.angle_max - self.angle_min) / self.angle_increment) + 1

        # Publishers
        self.lidar_pub = rospy.Publisher('/robot/laser_scan', LaserScan, queue_size=10)

        # Simulated environment representation
        self.environment_objects = []  # Static objects in environment
        self.dynamic_objects = []      # Moving objects (humans, etc.)

    def add_environment_object(self, center, dimensions):
        """Add a static object to the simulated environment"""
        self.environment_objects.append({
            'center': np.array(center),
            'dimensions': np.array(dimensions)  # [width, depth, height]
        })

    def simulate_scan(self):
        """Simulate a LiDAR scan based on environment objects"""
        # Generate angle array
        angles = np.linspace(self.angle_min, self.angle_max, self.num_rays)

        # Initialize ranges array
        ranges = np.full(self.num_rays, self.range_max)

        # Simulate ray casting to environment objects
        for i, angle in enumerate(angles):
            # Calculate ray direction in robot's local frame
            ray_direction = np.array([np.cos(angle), np.sin(angle)])

            # Calculate minimum distance to any object
            min_distance = self.range_max

            # Check against environment objects
            for obj in self.environment_objects:
                distance = self.cast_ray_to_box(ray_direction, obj)
                if distance > 0 and distance < min_distance:
                    min_distance = distance

            # Apply noise to the measurement
            noisy_distance = min_distance + np.random.normal(0, self.noise_std_dev)

            # Clamp to valid range
            ranges[i] = max(self.range_min, min(self.range_max, noisy_distance))

        return ranges

    def cast_ray_to_box(self, ray_direction, box):
        """Cast a ray to a 3D box and return intersection distance"""
        # Simplified 2D ray-box intersection for LiDAR simulation
        # In reality, this would be more complex for 3D objects

        # Box center and half-dimensions
        center = box['center'][:2]  # Use x,y coordinates
        half_dims = box['dimensions'][:2] / 2

        # Calculate ray-box intersection
        # Using slab method for ray-box intersection
        t_min = 0.0
        t_max = self.range_max

        ray_origin = np.array([0.0, 0.0])  # Robot is at origin

        for i in range(2):  # x and y axes
            if abs(ray_direction[i]) < 1e-6:  # Ray is parallel to slab
                if ray_origin[i] < center[i] - half_dims[i] or ray_origin[i] > center[i] + half_dims[i]:
                    return -1  # No intersection
            else:
                # Calculate intersection distances
                inv_dir = 1.0 / ray_direction[i]
                t1 = (center[i] - half_dims[i] - ray_origin[i]) * inv_dir
                t2 = (center[i] + half_dims[i] - ray_origin[i]) * inv_dir

                # Ensure t1 is smaller than t2
                if t1 > t2:
                    t1, t2 = t2, t1

                # Update interval for current axis
                t_min = max(t_min, t1)
                t_max = min(t_max, t2)

                if t_min > t_max:
                    return -1  # No intersection

        # Return the closest intersection distance
        return t_min if t_min >= 0 else t_max if t_max >= 0 else -1

    def publish_scan(self):
        """Publish simulated LiDAR scan"""
        # Simulate the scan
        ranges = self.simulate_scan()

        # Create LaserScan message
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = 'laser_frame'

        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0  # Not used in simulation
        scan_msg.scan_time = self.scan_time
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max

        scan_msg.ranges = ranges.tolist()
        scan_msg.intensities = []  # Intensity not simulated

        # Publish the scan
        self.lidar_pub.publish(scan_msg)

    def run(self):
        """Main simulation loop"""
        rate = rospy.Rate(10)  # 10 Hz

        # Add some example environment objects
        self.add_environment_object([2.0, 0.0], [0.5, 0.5, 2.0])  # Wall at 2m
        self.add_environment_object([1.5, 1.0], [0.3, 0.3, 1.5])  # Object at 1.5m, 1m

        while not rospy.is_shutdown():
            self.publish_scan()
            rate.sleep()

if __name__ == '__main__':
    lidar_sim = LiDARSensorSimulator()
    rospy.loginfo("LiDAR simulator started")
    lidar_sim.run()
```

## Depth Camera Simulation

### Depth Camera Configuration in Gazebo

```xml
<!-- Example depth camera configuration in Gazebo -->
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees in radians -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
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

### Depth Camera Simulator Implementation

```python
#!/usr/bin/env python3
"""
Depth camera simulation for humanoid robot digital twin
"""

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge
from std_msgs.msg import Header

class DepthCameraSimulator:
    """Simulates depth camera for humanoid robot digital twin"""

    def __init__(self):
        rospy.init_node('depth_camera_simulator')

        # Camera configuration
        self.width = 640
        self.height = 480
        self.fov_horizontal = 60  # degrees
        self.fov_vertical = 45    # degrees
        self.focal_length = 320   # pixels
        self.cx = self.width / 2  # principal point x
        self.cy = self.height / 2 # principal point y
        self.min_depth = 0.1      # meters
        self.max_depth = 10.0     # meters
        self.update_rate = 30     # Hz

        # Noise parameters
        self.depth_noise_std = 0.01  # meters

        # Publishers
        self.bridge = CvBridge()
        self.rgb_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)
        self.info_pub = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=10)

        # Simulated environment
        self.scene_objects = []  # 3D objects in the scene

    def add_scene_object(self, center, dimensions, shape='box'):
        """Add an object to the simulated scene"""
        self.scene_objects.append({
            'center': np.array(center),
            'dimensions': np.array(dimensions),
            'shape': shape
        })

    def simulate_depth_image(self):
        """Simulate depth image based on scene objects"""
        # Initialize depth image
        depth_image = np.full((self.height, self.width), self.max_depth, dtype=np.float32)

        # Calculate pixel coordinates
        u_coords, v_coords = np.meshgrid(np.arange(self.width), np.arange(self.height))

        # Convert to normalized image coordinates
        x_norm = (u_coords - self.cx) / self.focal_length
        y_norm = (v_coords - self.cy) / self.focal_length

        # Calculate ray directions (assuming z-forward in camera frame)
        ray_directions = np.stack([
            x_norm.flatten(),
            y_norm.flatten(),
            np.ones(u_coords.size)
        ], axis=1)
        ray_directions = ray_directions / np.linalg.norm(ray_directions, axis=1, keepdims=True)

        # For each ray, find the closest intersection with scene objects
        for i in range(ray_directions.shape[0]):
            ray_dir = ray_directions[i]

            # Find closest intersection with any object
            min_distance = self.max_depth

            for obj in self.scene_objects:
                distance = self.intersect_ray_with_object(ray_dir, obj)
                if distance > 0 and distance < min_distance:
                    min_distance = distance

            # Set depth value with noise
            pixel_idx = i
            v = pixel_idx // self.width
            u = pixel_idx % self.width
            depth_image[v, u] = min_distance + np.random.normal(0, self.depth_noise_std)

        # Clamp to valid range
        depth_image = np.clip(depth_image, self.min_depth, self.max_depth)

        return depth_image

    def intersect_ray_with_object(self, ray_direction, obj):
        """Intersect a ray with a 3D object (simplified for box shape)"""
        # Simplified ray-box intersection in camera coordinate system
        # Camera is at origin looking along positive Z

        # Convert object center to camera frame (simplified)
        obj_center = obj['center']
        half_dims = obj['dimensions'] / 2

        # Calculate ray-box intersection using slab method
        t_min = 0.0
        t_max = self.max_depth

        ray_origin = np.array([0.0, 0.0, 0.0])  # Camera at origin

        for i in range(3):  # x, y, z axes
            if abs(ray_direction[i]) < 1e-6:  # Ray is parallel to slab
                if ray_origin[i] < obj_center[i] - half_dims[i] or ray_origin[i] > obj_center[i] + half_dims[i]:
                    return -1  # No intersection
            else:
                # Calculate intersection distances
                inv_dir = 1.0 / ray_direction[i]
                t1 = (obj_center[i] - half_dims[i] - ray_origin[i]) * inv_dir
                t2 = (obj_center[i] + half_dims[i] - ray_origin[i]) * inv_dir

                # Ensure t1 is smaller than t2
                if t1 > t2:
                    t1, t2 = t2, t1

                # Update interval for current axis
                t_min = max(t_min, t1)
                t_max = min(t_max, t2)

                if t_min > t_max:
                    return -1  # No intersection

        # Return the closest positive intersection distance
        if t_min >= 0:
            return t_min
        elif t_max >= 0:
            return t_max
        else:
            return -1  # No valid intersection

    def simulate_rgb_image(self, depth_image):
        """Simulate RGB image based on depth information"""
        # In a real simulation, this would render based on scene objects
        # For this example, we'll create a simple visualization

        # Create a pseudo-color image based on depth
        normalized_depth = (depth_image - self.min_depth) / (self.max_depth - self.min_depth)
        normalized_depth = np.clip(normalized_depth, 0, 1)

        # Convert to RGB using a colormap
        depth_colormap = cv2.applyColorMap((normalized_depth * 255).astype(np.uint8), cv2.COLORMAP_JET)

        return depth_colormap

    def publish_images(self):
        """Publish simulated RGB and depth images"""
        # Simulate depth image
        depth_image = self.simulate_depth_image()

        # Simulate RGB image
        rgb_image = self.simulate_rgb_image(depth_image)

        # Publish depth image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
        depth_msg.header = Header()
        depth_msg.header.stamp = rospy.Time.now()
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        self.depth_pub.publish(depth_msg)

        # Publish RGB image
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
        rgb_msg.header = Header()
        rgb_msg.header.stamp = rospy.Time.now()
        rgb_msg.header.frame_id = 'camera_rgb_optical_frame'
        self.rgb_pub.publish(rgb_msg)

        # Publish camera info
        self.publish_camera_info()

    def publish_camera_info(self):
        """Publish camera information"""
        info_msg = CameraInfo()
        info_msg.header = Header()
        info_msg.header.stamp = rospy.Time.now()
        info_msg.header.frame_id = 'camera_rgb_optical_frame'

        info_msg.width = self.width
        info_msg.height = self.height
        info_msg.distortion_model = 'plumb_bob'

        # Camera intrinsic matrix
        info_msg.K = [
            self.focal_length, 0.0, self.cx,
            0.0, self.focal_length, self.cy,
            0.0, 0.0, 1.0
        ]

        # Distortion coefficients (none for simplicity)
        info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Rectification matrix (identity for monocular camera)
        info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Projection matrix
        info_msg.P = [
            self.focal_length, 0.0, self.cx, 0.0,
            0.0, self.focal_length, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        self.info_pub.publish(info_msg)

    def run(self):
        """Main simulation loop"""
        rate = rospy.Rate(self.update_rate)

        # Add some example scene objects
        self.add_scene_object([2.0, 0.0, 1.0], [0.5, 0.5, 2.0])  # Wall at 2m
        self.add_scene_object([1.0, -0.5, 0.8], [0.3, 0.3, 1.0])  # Object closer

        while not rospy.is_shutdown():
            self.publish_images()
            rate.sleep()

if __name__ == '__main__':
    depth_cam = DepthCameraSimulator()
    rospy.loginfo("Depth camera simulator started")
    depth_cam.run()
```

## IMU Simulation

### IMU Configuration in Gazebo

```xml
<!-- Example IMU sensor configuration in Gazebo -->
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>  <!-- 100 Hz update rate -->
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>  <!-- ~0.1 deg/s stddev -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0</bias_stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>  <!-- ~0.017 m/s² stddev -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
    <topicName>/robot/imu/data</topicName>
    <serviceName>/robot/imu/service</serviceName>
    <gaussianNoise>0.0</gaussianNoise>
    <frameName>imu_frame</frameName>
  </plugin>
</sensor>
```

### IMU Simulator Implementation

```python
#!/usr/bin/env python3
"""
IMU sensor simulation for humanoid robot digital twin
"""

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import tf.transformations as tft

class IMUSimulator:
    """Simulates IMU sensor for humanoid robot digital twin"""

    def __init__(self):
        rospy.init_node('imu_simulator')

        # IMU configuration
        self.update_rate = 100  # Hz
        self.gravity = 9.81     # m/s²

        # Noise parameters (typical for consumer-grade IMUs)
        self.accel_noise_std = 0.017    # m/s²
        self.gyro_noise_std = 0.0017   # rad/s (~0.1 deg/s)
        self.accel_bias_std = 0.001    # m/s² bias drift
        self.gyro_bias_std = 0.00017   # rad/s bias drift (~0.01 deg/s)

        # True state (for simulation)
        self.true_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # [x, y, z, w] quaternion
        self.true_angular_velocity = np.array([0.0, 0.0, 0.0])  # rad/s
        self.true_linear_acceleration = np.array([0.0, 0.0, 0.0])  # m/s² (excluding gravity)

        # Bias tracking
        self.accel_bias = np.random.normal(0, self.accel_bias_std, 3)
        self.gyro_bias = np.random.normal(0, self.gyro_bias_std, 3)

        # Publishers
        self.imu_pub = rospy.Publisher('/robot/imu/data', Imu, queue_size=10)

        # Timing
        self.last_update_time = rospy.Time.now()

    def update_true_state(self):
        """Update the true state of the IMU based on robot dynamics"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_update_time).to_sec()
        self.last_update_time = current_time

        if dt <= 0:
            return

        # For this simulation, we'll simulate simple dynamics
        # In a real system, this would come from physics simulation
        # Add some simple oscillatory motion to make it interesting
        t = current_time.to_sec()

        # Simulate small oscillations
        oscillation_freq = 0.5  # Hz
        oscillation_amplitude = 0.1  # rad/s for angular velocity

        # Update angular velocity with oscillation
        self.true_angular_velocity[0] = 0.01 * np.sin(2 * np.pi * oscillation_freq * t)
        self.true_angular_velocity[1] = 0.01 * np.cos(2 * np.pi * oscillation_freq * t)
        self.true_angular_velocity[2] = 0.02 * np.sin(2 * np.pi * oscillation_freq * t + np.pi/4)

        # Update orientation based on angular velocity
        # Simple quaternion integration (first-order)
        omega_quat = np.array([
            self.true_angular_velocity[0],
            self.true_angular_velocity[1],
            self.true_angular_velocity[2],
            0.0
        ])

        # dq/dt = 0.5 * q ⊗ ω (quaternion derivative)
        quat_derivative = 0.5 * tft.quaternion_multiply(self.true_orientation, omega_quat)
        self.true_orientation += quat_derivative * dt

        # Normalize quaternion
        self.true_orientation /= np.linalg.norm(self.true_orientation)

        # Simulate linear acceleration (due to motion)
        # This would come from physics simulation in a real system
        self.true_linear_acceleration[0] = 0.05 * np.sin(2 * np.pi * 0.3 * t)
        self.true_linear_acceleration[1] = 0.03 * np.cos(2 * np.pi * 0.3 * t)
        self.true_linear_acceleration[2] = 0.0  # Mostly balanced

    def simulate_imu_reading(self):
        """Simulate IMU reading with realistic noise and biases"""
        # Update true state
        self.update_true_state()

        # Simulate accelerometer reading
        # True acceleration + gravity + noise + bias
        gravity_vector = self.rotate_vector_to_body_frame(np.array([0, 0, -self.gravity]))
        true_accel_with_gravity = self.true_linear_acceleration + gravity_vector

        accel_measurement = (
            true_accel_with_gravity +
            np.random.normal(0, self.accel_noise_std, 3) +
            self.accel_bias
        )

        # Simulate gyroscope reading
        gyro_measurement = (
            self.true_angular_velocity +
            np.random.normal(0, self.gyro_noise_std, 3) +
            self.gyro_bias
        )

        # Update biases (slow drift)
        self.accel_bias += np.random.normal(0, self.accel_bias_std * 0.01, 3)
        self.gyro_bias += np.random.normal(0, self.gyro_bias_std * 0.01, 3)

        return accel_measurement, gyro_measurement

    def rotate_vector_to_body_frame(self, world_vector):
        """Rotate a vector from world frame to body frame using current orientation"""
        # Convert quaternion to rotation matrix
        rotation_matrix = tft.quaternion_matrix(self.true_orientation)[:3, :3]

        # Rotate vector
        body_vector = np.dot(rotation_matrix.T, world_vector)
        return body_vector

    def publish_imu_data(self):
        """Publish simulated IMU data"""
        # Simulate IMU readings
        accel, gyro = self.simulate_imu_reading()

        # Create IMU message
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu_frame'

        # Set orientation (in this simulation, we'll publish zeros since we need to estimate from dynamics)
        # In a real system, magnetometers would help estimate absolute orientation
        imu_msg.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Indicates no orientation data

        # Set angular velocity
        imu_msg.angular_velocity = Vector3(gyro[0], gyro[1], gyro[2])
        imu_msg.angular_velocity_covariance = [
            self.gyro_noise_std**2, 0.0, 0.0,
            0.0, self.gyro_noise_std**2, 0.0,
            0.0, 0.0, self.gyro_noise_std**2
        ]

        # Set linear acceleration
        imu_msg.linear_acceleration = Vector3(accel[0], accel[1], accel[2])
        imu_msg.linear_acceleration_covariance = [
            self.accel_noise_std**2, 0.0, 0.0,
            0.0, self.accel_noise_std**2, 0.0,
            0.0, 0.0, self.accel_noise_std**2
        ]

        # Publish the message
        self.imu_pub.publish(imu_msg)

    def run(self):
        """Main simulation loop"""
        rate = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            self.publish_imu_data()
            rate.sleep()

if __name__ == '__main__':
    imu_sim = IMUSimulator()
    rospy.loginfo("IMU simulator started")
    imu_sim.run()
```

## Sensor Fusion in Digital Twins

### Multi-Sensor Integration

```python
#!/usr/bin/env python3
"""
Multi-sensor fusion for humanoid robot digital twin
"""

import rospy
import numpy as np
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf

class SensorFusionNode:
    """Fuses multiple sensors for improved state estimation in digital twin"""

    def __init__(self):
        rospy.init_node('sensor_fusion_node')

        # Subscribe to sensor data
        self.imu_sub = rospy.Subscriber('/robot/imu/data', Imu, self.imu_callback)
        self.lidar_sub = rospy.Subscriber('/robot/laser_scan', LaserScan, self.lidar_callback)

        # Publisher for fused state
        self.pose_pub = rospy.Publisher('/robot/pose_estimate', PoseWithCovarianceStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/robot/odom', Odometry, queue_size=10)

        # Internal state
        self.current_pose = np.zeros(3)  # x, y, theta
        self.current_velocity = np.zeros(3)  # vx, vy, omega
        self.state_covariance = np.eye(6) * 0.1  # Initial covariance

        # Timing
        self.last_update_time = rospy.Time.now()

        # Sensor noise characteristics
        self.imu_process_noise = 0.01
        self.odometry_process_noise = 0.05
        self.measurement_noise = 0.1

    def imu_callback(self, msg):
        """Process IMU data for state prediction"""
        # Extract angular velocity for pose integration
        angular_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Extract linear acceleration
        linear_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Update state based on IMU data
        self.predict_state_imu(angular_vel, linear_acc)

    def lidar_callback(self, msg):
        """Process LiDAR data for state correction"""
        # In a real implementation, this would extract features from the scan
        # and use them to correct the pose estimate
        # For this example, we'll just use it as a trigger for correction
        self.correct_state_lidar(msg)

    def predict_state_imu(self, angular_vel, linear_acc):
        """Predict state using IMU data"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_update_time).to_sec()
        self.last_update_time = current_time

        if dt <= 0:
            return

        # Integrate angular velocity to get orientation change
        delta_theta = angular_vel[2] * dt  # Assuming mostly z-axis rotation

        # Update orientation
        self.current_pose[2] += delta_theta

        # Transform acceleration to global frame and integrate
        cos_th = np.cos(self.current_pose[2])
        sin_th = np.sin(self.current_pose[2])

        global_acc = np.array([
            linear_acc[0] * cos_th - linear_acc[1] * sin_th,
            linear_acc[0] * sin_th + linear_acc[1] * cos_th,
            0  # z-axis acceleration doesn't affect 2D pose
        ])

        # Update velocity
        self.current_velocity[0] += global_acc[0] * dt
        self.current_velocity[1] += global_acc[1] * dt
        self.current_velocity[2] = delta_theta / dt if dt > 0.001 else 0

        # Update position
        self.current_pose[0] += self.current_velocity[0] * dt
        self.current_pose[1] += self.current_velocity[1] * dt

        # Propagate uncertainty (process noise)
        self.propagate_uncertainty(dt)

    def correct_state_lidar(self, scan_msg):
        """Correct state estimate using LiDAR data"""
        # In a real implementation, this would:
        # 1. Extract features from the LiDAR scan
        # 2. Match features to map
        # 3. Compute correction based on feature correspondences
        #
        # For this example, we'll just reduce uncertainty
        # since LiDAR provides good positional information
        self.state_covariance *= 0.9  # Reduce uncertainty

    def propagate_uncertainty(self, dt):
        """Propagate state uncertainty through time"""
        # Simple Jacobian for state transition
        F = np.eye(6)  # Linearized state transition matrix
        F[0, 3] = dt  # dx/dvx = dt
        F[1, 4] = dt  # dy/dvy = dt
        F[2, 5] = dt  # dtheta/domega = dt

        # Process noise matrix (simplified)
        Q = np.diag([
            self.odometry_process_noise**2 * dt,
            self.odometry_process_noise**2 * dt,
            self.imu_process_noise**2 * dt,  # orientation uncertainty
            self.odometry_process_noise**2,
            self.odometry_process_noise**2,
            self.imu_process_noise**2  # angular velocity uncertainty
        ])

        # Propagate covariance: P = F*P*F^T + Q
        self.state_covariance = F @ self.state_covariance @ F.T + Q

        # Keep covariance bounded
        self.state_covariance = np.clip(self.state_covariance, 1e-6, 1e6)

    def publish_fused_state(self):
        """Publish the fused state estimate"""
        # Create pose with covariance message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.pose.position.x = self.current_pose[0]
        pose_msg.pose.pose.position.y = self.current_pose[1]
        pose_msg.pose.pose.position.z = 0.0

        # Convert 2D pose to quaternion
        quat = tf.transformations.quaternion_from_euler(0, 0, self.current_pose[2])
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # Flatten covariance matrix
        pose_msg.pose.covariance = self.state_covariance.flatten()

        # Publish pose
        self.pose_pub.publish(pose_msg)

        # Also publish odometry message
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose = pose_msg.pose  # Copy pose data

        # Set velocity
        odom_msg.twist.twist.linear.x = self.current_velocity[0]
        odom_msg.twist.twist.linear.y = self.current_velocity[1]
        odom_msg.twist.twist.angular.z = self.current_velocity[2]

        # Set velocity covariances
        vel_cov = np.eye(6) * 0.01  # Placeholder covariance
        odom_msg.twist.covariance = vel_cov.flatten()

        self.odom_pub.publish(odom_msg)

    def run(self):
        """Main fusion loop"""
        rate = rospy.Rate(50)  # 50 Hz fusion rate

        while not rospy.is_shutdown():
            self.publish_fused_state()
            rate.sleep()

if __name__ == '__main__':
    fusion_node = SensorFusionNode()
    rospy.loginfo("Sensor fusion node started")
    fusion_node.run()
```

## Sensor Validation and Calibration

### Simulation Validation Techniques

```python
#!/usr/bin/env python3
"""
Sensor validation and calibration for digital twin systems
"""

import rospy
import numpy as np
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64
import matplotlib.pyplot as plt

class SensorValidator:
    """Validates sensor simulation accuracy"""

    def __init__(self):
        rospy.init_node('sensor_validator')

        # Subscribe to sensor data
        self.imu_sub = rospy.Subscriber('/robot/imu/data', Imu, self.imu_validation_callback)
        self.lidar_sub = rospy.Subscriber('/robot/laser_scan', LaserScan, self.lidar_validation_callback)

        # Publishers for validation metrics
        self.accuracy_pub = rospy.Publisher('/sensor_validation/accuracy', Float64, queue_size=10)

        # Validation buffers
        self.imu_buffer = []
        self.lidar_buffer = []
        self.buffer_size = 1000

        # Validation parameters
        self.gravity_tolerance = 0.1  # m/s²
        self.range_tolerance = 0.05   # meters

    def imu_validation_callback(self, msg):
        """Validate IMU readings"""
        # Check if gravity is properly sensed when robot is stationary
        linear_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Calculate magnitude of sensed gravity
        sensed_gravity = np.linalg.norm(linear_acc)

        # Validate gravity sensing (should be ~9.81 m/s² when stationary)
        gravity_error = abs(sensed_gravity - 9.81)

        # Store validation result
        validation_result = {
            'timestamp': rospy.Time.now(),
            'gravity_error': gravity_error,
            'linear_acc': linear_acc,
            'angular_vel': np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        }

        self.imu_buffer.append(validation_result)
        if len(self.imu_buffer) > self.buffer_size:
            self.imu_buffer.pop(0)

        # Calculate and publish accuracy metric
        accuracy_metric = Float64()
        accuracy_metric.data = max(0.0, 1.0 - gravity_error / 10.0)  # Normalize to 0-1 range
        self.accuracy_pub.publish(accuracy_metric)

        # Log validation results periodically
        if len(self.imu_buffer) % 100 == 0:
            self.log_imu_validation()

    def lidar_validation_callback(self, msg):
        """Validate LiDAR readings"""
        # Check for realistic range values
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]

        # Store validation result
        validation_result = {
            'timestamp': rospy.Time.now(),
            'valid_ranges_ratio': len(valid_ranges) / len(ranges) if len(ranges) > 0 else 0,
            'mean_range': np.mean(valid_ranges) if len(valid_ranges) > 0 else 0,
            'std_range': np.std(valid_ranges) if len(valid_ranges) > 0 else 0
        }

        self.lidar_buffer.append(validation_result)
        if len(self.lidar_buffer) > self.buffer_size:
            self.lidar_buffer.pop(0)

        # Log validation results periodically
        if len(self.lidar_buffer) % 100 == 0:
            self.log_lidar_validation()

    def log_imu_validation(self):
        """Log IMU validation statistics"""
        if not self.imu_buffer:
            return

        gravity_errors = [item['gravity_error'] for item in self.imu_buffer]
        avg_error = np.mean(gravity_errors)
        max_error = np.max(gravity_errors)

        rospy.loginfo(f"IMU Validation - Avg gravity error: {avg_error:.3f} m/s², Max: {max_error:.3f} m/s²")

        # Check if validation is passing
        if avg_error > self.gravity_tolerance:
            rospy.logwarn(f"IMU gravity sensing accuracy degrading. Avg error: {avg_error:.3f} m/s² > tolerance: {self.gravity_tolerance}")

    def log_lidar_validation(self):
        """Log LiDAR validation statistics"""
        if not self.lidar_buffer:
            return

        valid_ratios = [item['valid_ranges_ratio'] for item in self.lidar_buffer]
        mean_valid_ratio = np.mean(valid_ratios)

        rospy.loginfo(f"LiDAR Validation - Avg valid ranges: {mean_valid_ratio:.2%}")

        # Check if validation is passing
        if mean_valid_ratio < 0.9:  # Less than 90% valid ranges
            rospy.logwarn(f"LiDAR range validation failing. Valid ratio: {mean_valid_ratio:.2%}")

    def generate_validation_report(self):
        """Generate a comprehensive validation report"""
        rospy.loginfo("Generating sensor validation report...")

        # IMU validation report
        if self.imu_buffer:
            gravity_errors = [item['gravity_error'] for item in self.imu_buffer]
            angular_velocities = [item['angular_vel'] for item in self.imu_buffer]

            avg_gravity_error = np.mean(gravity_errors)
            std_gravity_error = np.std(gravity_errors)
            avg_angular_vel = np.mean(angular_velocities, axis=0)

            rospy.loginfo(f"\n=== IMU Validation Report ===")
            rospy.loginfo(f"Average gravity error: {avg_gravity_error:.3f} ± {std_gravity_error:.3f} m/s²")
            rospy.loginfo(f"Average angular velocity: {avg_angular_vel}")

        # LiDAR validation report
        if self.lidar_buffer:
            valid_ratios = [item['valid_ranges_ratio'] for item in self.lidar_buffer]
            mean_ranges = [item['mean_range'] for item in self.lidar_buffer]

            avg_valid_ratio = np.mean(valid_ratios)
            avg_mean_range = np.mean(mean_ranges)

            rospy.loginfo(f"\n=== LiDAR Validation Report ===")
            rospy.loginfo(f"Average valid range ratio: {avg_valid_ratio:.2%}")
            rospy.loginfo(f"Average measured range: {avg_mean_range:.2f} m")

        rospy.loginfo(f"\nValidation report complete.")

    def run(self):
        """Main validation loop"""
        rate = rospy.Rate(1)  # Log validation once per second

        while not rospy.is_shutdown():
            rate.sleep()

        # Generate final report on shutdown
        self.generate_validation_report()

if __name__ == '__main__':
    validator = SensorValidator()
    rospy.loginfo("Sensor validator started")
    validator.run()
```

## Summary

Sensor simulation is critical for creating effective digital twins of humanoid robots. Realistic simulation of LiDAR, depth cameras, and IMUs enables comprehensive testing and training of perception and control systems before deployment on physical robots. Proper validation and calibration of sensor simulations ensure that the digital twin accurately reflects the behavior of its physical counterpart.

## Next Steps

Continue to learn about implementing learning objectives and outcomes for the sensor simulation chapter.

[← Previous: Unity Navigation](./unity-navigation) | [Next: Sensor Learning Objectives →](./sensor-learning-objectives)