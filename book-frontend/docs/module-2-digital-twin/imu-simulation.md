---
sidebar_position: 21
---

# IMU Simulation for Humanoid Robots

## Learning Objectives

By the end of this section, you will be able to:
- Configure realistic IMU sensors in Gazebo simulation environments
- Understand IMU principles and sensor fusion techniques
- Implement IMU noise models that match real-world characteristics
- Process IMU data for orientation estimation and state tracking
- Validate IMU simulation accuracy for digital twin applications

## Overview

Inertial Measurement Units (IMUs) are critical sensors for humanoid robots, providing essential data about orientation, angular velocity, and linear acceleration. In digital twin environments, realistic IMU simulation is crucial for accurate state estimation, balance control, and navigation. This section covers the principles, configuration, and implementation of IMU simulation for humanoid robot digital twins.

## IMU Fundamentals

### How IMUs Work

IMUs combine multiple sensors to provide comprehensive motion data:

- **Accelerometer**: Measures linear acceleration along 3 axes (x, y, z)
- **Gyroscope**: Measures angular velocity around 3 axes (roll, pitch, yaw)
- **Magnetometer**: Measures magnetic field strength for heading reference (optional)

The combination of these measurements allows for:
- **Orientation Estimation**: Determining robot attitude in 3D space
- **Motion Tracking**: Monitoring robot movement and acceleration
- **Balance Control**: Providing feedback for balance and stability algorithms
- **Navigation**: Supporting dead reckoning and localization

### IMU Specifications and Parameters

Key parameters that affect IMU performance:

- **Measurement Range**: Maximum measurable acceleration/angular velocity
- **Resolution**: Smallest detectable change in measurement
- **Noise Density**: Noise level per square root of bandwidth
- **Bias Stability**: Long-term stability of sensor bias
- **Scale Factor Error**: Gain error in measurements
- **Cross-Axis Sensitivity**: Coupling between different axes
- **Temperature Drift**: Change in characteristics with temperature
- **Bandwidth**: Frequency response of the sensor

## IMU Configuration in Gazebo

### Basic IMU Configuration

```xml
<!-- Example: Basic IMU configuration -->
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
          <stddev>0.0017</stddev>  <!-- ~0.1 deg/s stddev -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>  <!-- ~0.1 deg/s stddev -->
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
          <stddev>0.017</stddev>  <!-- ~0.017 m/s² stddev -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.0</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>  <!-- ~0.017 m/s² stddev -->
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

### Advanced IMU Configuration

```xml
<!-- Example: Advanced IMU with realistic noise and bias models -->
<sensor name="advanced_imu" type="imu">
  <always_on>1</always_on>
  <update_rate>200</update_rate>  <!-- 200 Hz for high-performance IMU -->
  <topic>imu/data</topic>

  <imu>
    <!-- Angular velocity parameters -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.00087</stddev>  <!-- ~0.05 deg/s stddev (high-grade IMU) -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.00017</bias_stddev>  <!-- ~0.01 deg/s bias drift -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.00087</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.00017</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.00087</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.00017</bias_stddev>
        </noise>
      </z>
    </angular_velocity>

    <!-- Linear acceleration parameters -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.008</stddev>  <!-- ~8 mg noise (high-grade accelerometer) -->
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.001</bias_stddev>  <!-- ~1 mg bias drift -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.008</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.008</stddev>
          <bias_mean>0.0</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>

    <!-- Orientation (magnetometer) - optional -->
    <orientation>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>  <!-- ~0.1 deg noise -->
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0017</stddev>
        </noise>
      </z>
    </orientation>
  </imu>

  <!-- Advanced plugin configuration -->
  <plugin name="advanced_imu_controller" filename="libgazebo_ros_imu.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>200.0</updateRate>
    <bodyName>imu_link</bodyName>
    <topicName>/robot/imu/data</topicName>
    <serviceName>/robot/imu/service</serviceName>
    <gaussianNoise>0.0</gaussianNoise>
    <frameName>imu_link</frameName>
    <initialOrientationAsReference>false</initialOrientationAsReference>
    <xyzOffset>0 0 0</xyzOffset>
    <rpyOffset>0 0 0</rpyOffset>
    <pubRate>200.0</pubRate>
  </plugin>
</sensor>
```

## IMU Data Processing

### IMU Data Processing Pipeline

```python
#!/usr/bin/env python3
"""
IMU data processing pipeline for humanoid robot digital twin
"""

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
from scipy.spatial.transform import Rotation as R

class IMUProcessor:
    """Processes IMU data for humanoid robot applications"""

    def __init__(self):
        rospy.init_node('imu_processor')

        # Publishers
        self.processed_imu_pub = rospy.Publisher('/robot/imu/processed', Imu, queue_size=10)
        self.orientation_pub = rospy.Publisher('/robot/orientation', Quaternion, queue_size=10)
        self.angular_velocity_pub = rospy.Publisher('/robot/angular_velocity', Vector3, queue_size=10)
        self.linear_acceleration_pub = rospy.Publisher('/robot/linear_acceleration', Vector3, queue_size=10)

        # Subscribers
        self.imu_sub = rospy.Subscriber('/robot/imu/data', Imu, self.imu_callback)

        # IMU configuration parameters
        self.gravity = 9.81  # m/s²
        self.update_rate = 100  # Hz

        # Noise parameters (typical for consumer-grade IMUs)
        self.accel_noise_std = 0.017  # m/s²
        self.gyro_noise_std = 0.0017  # rad/s (~0.1 deg/s)
        self.accel_bias_std = 0.001   # m/s² bias drift
        self.gyro_bias_std = 0.00017  # rad/s bias drift (~0.01 deg/s)

        # State estimation
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # [x, y, z, w] quaternion
        self.angular_velocity = np.array([0.0, 0.0, 0.0])  # rad/s
        self.linear_acceleration = np.array([0.0, 0.0, 0.0])  # m/s²
        self.velocity = np.array([0.0, 0.0, 0.0])  # m/s
        self.position = np.array([0.0, 0.0, 0.0])  # m

        # Bias tracking
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

        # Kalman filter parameters for sensor fusion
        self.kalman_process_noise = 0.1
        self.kalman_measurement_noise = 0.1
        self.kalman_error_covariance = np.eye(6) * 0.1

        # Timing
        self.last_update_time = rospy.Time.now()

    def imu_callback(self, imu_msg):
        """Process incoming IMU data"""
        try:
            # Extract raw measurements
            raw_angular_vel = np.array([
                imu_msg.angular_velocity.x,
                imu_msg.angular_velocity.y,
                imu_msg.angular_velocity.z
            ])

            raw_linear_acc = np.array([
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z
            ])

            # Get timestamp
            current_time = imu_msg.header.stamp
            dt = (current_time - self.last_update_time).to_sec() if self.last_update_time else 0
            self.last_update_time = current_time

            if dt <= 0:
                return

            # Apply bias correction and noise filtering
            corrected_angular_vel = self.correct_angular_velocity(raw_angular_vel, dt)
            corrected_linear_acc = self.correct_linear_acceleration(raw_linear_acc, dt)

            # Update state estimation
            self.update_state_estimation(corrected_angular_vel, corrected_linear_acc, dt)

            # Publish processed data
            self.publish_processed_imu(imu_msg.header, corrected_angular_vel, corrected_linear_acc)
            self.publish_orientation(imu_msg.header)

        except Exception as e:
            rospy.logerr(f"Error processing IMU data: {e}")

    def correct_angular_velocity(self, raw_angular_vel, dt):
        """Apply bias correction and filtering to angular velocity"""
        # Subtract estimated bias
        corrected_angular_vel = raw_angular_vel - self.gyro_bias

        # Apply low-pass filter to reduce noise
        alpha = 0.1  # Filter coefficient
        self.angular_velocity = alpha * corrected_angular_vel + (1 - alpha) * self.angular_velocity

        # Update bias estimate (slow drift)
        self.gyro_bias += np.random.normal(0, self.gyro_bias_std * dt * 0.01, 3)

        return self.angular_velocity

    def correct_linear_acceleration(self, raw_linear_acc, dt):
        """Apply bias correction and filtering to linear acceleration"""
        # Subtract estimated bias
        corrected_linear_acc = raw_linear_acc - self.accel_bias

        # Apply low-pass filter to reduce noise
        alpha = 0.1  # Filter coefficient
        self.linear_acceleration = alpha * corrected_linear_acc + (1 - alpha) * self.linear_acceleration

        # Update bias estimate (slow drift)
        self.accel_bias += np.random.normal(0, self.accel_bias_std * dt * 0.01, 3)

        return self.linear_acceleration

    def update_state_estimation(self, angular_vel, linear_acc, dt):
        """Update orientation, velocity, and position based on IMU measurements"""
        # Update orientation using quaternion integration
        self.update_orientation(angular_vel, dt)

        # Update velocity and position from linear acceleration
        self.update_motion_state(linear_acc, dt)

    def update_orientation(self, angular_vel, dt):
        """Update orientation using angular velocity integration"""
        # Convert angular velocity to quaternion derivative
        # q_dot = 0.5 * q ⊗ ω (quaternion derivative)
        omega_quat = np.array([angular_vel[0], angular_vel[1], angular_vel[2], 0.0])
        quat_derivative = 0.5 * self.quaternion_multiply(self.orientation, omega_quat)

        # Integrate quaternion
        self.orientation += quat_derivative * dt

        # Normalize quaternion to maintain unit length
        norm = np.linalg.norm(self.orientation)
        if norm > 0:
            self.orientation /= norm

    def update_motion_state(self, linear_acc, dt):
        """Update velocity and position from linear acceleration"""
        # Transform acceleration from body frame to world frame
        world_acc = self.rotate_vector_to_world_frame(linear_acc)

        # Remove gravity from linear acceleration
        gravity_vector = np.array([0, 0, self.gravity])
        linear_world_acc = world_acc - gravity_vector

        # Integrate to get velocity and position
        self.velocity += linear_world_acc * dt
        self.position += self.velocity * dt + 0.5 * linear_world_acc * dt**2

    def rotate_vector_to_world_frame(self, body_vector):
        """Rotate a vector from body frame to world frame using current orientation"""
        # Convert quaternion to rotation matrix
        rotation_matrix = self.quaternion_to_rotation_matrix(self.orientation)

        # Apply rotation
        world_vector = rotation_matrix @ body_vector
        return world_vector

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])

    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to 3x3 rotation matrix"""
        w, x, y, z = q

        # Rotation matrix from quaternion
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])

        return R

    def publish_processed_imu(self, header, corrected_angular_vel, corrected_linear_acc):
        """Publish processed IMU data"""
        imu_msg = Imu()
        imu_msg.header = header

        # Set orientation (if available from fusion)
        imu_msg.orientation = Quaternion(*self.orientation)

        # Set angular velocity
        imu_msg.angular_velocity = Vector3(*corrected_angular_vel)
        imu_msg.angular_velocity_covariance = [
            self.gyro_noise_std**2, 0.0, 0.0,
            0.0, self.gyro_noise_std**2, 0.0,
            0.0, 0.0, self.gyro_noise_std**2
        ]

        # Set linear acceleration
        imu_msg.linear_acceleration = Vector3(*corrected_linear_acc)
        imu_msg.linear_acceleration_covariance = [
            self.accel_noise_std**2, 0.0, 0.0,
            0.0, self.accel_noise_std**2, 0.0,
            0.0, 0.0, self.accel_noise_std**2
        ]

        self.processed_imu_pub.publish(imu_msg)

    def publish_orientation(self, header):
        """Publish orientation separately"""
        orientation_msg = Quaternion(*self.orientation)
        self.orientation_pub.publish(orientation_msg)

        # Publish angular velocity
        angular_vel_msg = Vector3(*self.angular_velocity)
        self.angular_velocity_pub.publish(angular_vel_msg)

        # Publish linear acceleration
        linear_acc_msg = Vector3(*self.linear_acceleration)
        self.linear_acceleration_pub.publish(linear_acc_msg)

    def run(self):
        """Main processing loop"""
        rospy.loginfo("IMU processor started")
        rospy.spin()

if __name__ == '__main__':
    processor = IMUProcessor()
    processor.run()
```

## Orientation Estimation

### Sensor Fusion for Orientation

```python
#!/usr/bin/env python3
"""
Orientation estimation using IMU sensor fusion
"""

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R

class OrientationEstimator:
    """Estimates orientation using IMU sensor fusion"""

    def __init__(self):
        rospy.init_node('orientation_estimator')

        # Publishers
        self.orientation_pub = rospy.Publisher('/robot/orientation_estimated', Imu, queue_size=10)

        # Subscribers
        self.imu_sub = rospy.Subscriber('/robot/imu/data', Imu, self.imu_callback)

        # State estimation parameters
        self.gravity = 9.81
        self.dt = 0.01  # 100 Hz (0.01 seconds)

        # Initial state
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # [x, y, z, w]
        self.bias = np.zeros(3)  # Gyro bias

        # Sensor fusion weights
        self.accel_weight = 0.1  # Weight for accelerometer correction
        self.mag_weight = 0.1    # Weight for magnetometer correction (if available)

        # Covariance matrices for Kalman filter
        self.process_noise = np.eye(4) * 0.001
        self.measurement_noise = np.eye(3) * 0.1

        # Timing
        self.last_time = rospy.Time.now()

    def imu_callback(self, imu_msg):
        """Process IMU data for orientation estimation"""
        # Extract measurements
        gyro = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])

        accel = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])

        # Get time delta
        current_time = imu_msg.header.stamp
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        if dt <= 0:
            return

        # Predict orientation using gyroscope
        predicted_orientation = self.predict_orientation(gyro, dt)

        # Correct orientation using accelerometer
        corrected_orientation = self.correct_with_accelerometer(
            predicted_orientation, accel, self.accel_weight
        )

        # Update internal state
        self.orientation = corrected_orientation

        # Publish estimated orientation
        self.publish_orientation(imu_msg.header)

    def predict_orientation(self, gyro, dt):
        """Predict orientation using gyroscope integration"""
        # Remove bias from gyro measurements
        corrected_gyro = gyro - self.bias

        # Convert angular velocity to quaternion derivative
        # q_dot = 0.5 * q ⊗ ω
        omega_quat = np.concatenate([corrected_gyro, [0.0]])
        quat_derivative = 0.5 * self.quaternion_multiply(self.orientation, omega_quat)

        # Integrate
        new_orientation = self.orientation + quat_derivative * dt

        # Normalize quaternion
        new_orientation = new_orientation / np.linalg.norm(new_orientation)

        return new_orientation

    def correct_with_accelerometer(self, predicted_orientation, accel, weight):
        """Correct orientation estimate using accelerometer data"""
        # Normalize accelerometer reading
        accel_norm = np.linalg.norm(accel)
        if accel_norm == 0:
            return predicted_orientation

        accel_unit = accel / accel_norm

        # Convert predicted orientation to expected gravity vector
        rotation_matrix = self.quaternion_to_rotation_matrix(predicted_orientation)

        # Expected gravity vector in sensor frame (should point opposite to gravity)
        expected_gravity = rotation_matrix.T @ np.array([0, 0, -1])

        # Calculate correction
        correction = np.cross(expected_gravity, accel_unit)

        # Apply correction with weight
        corrected_orientation = self.apply_correction_quaternion(
            predicted_orientation, correction, weight
        )

        return corrected_orientation

    def apply_correction_quaternion(self, orientation, correction, weight):
        """Apply small angle correction to quaternion"""
        # Convert small angle correction to quaternion
        angle = np.linalg.norm(correction) * weight

        if angle > 0:
            axis = correction / np.linalg.norm(correction)

            # Create correction quaternion: [axis*sin(angle/2), cos(angle/2)]
            sin_half = np.sin(angle / 2)
            cos_half = np.cos(angle / 2)

            correction_quat = np.array([
                axis[0] * sin_half,
                axis[1] * sin_half,
                axis[2] * sin_half,
                cos_half
            ])

            # Apply correction
            corrected_orientation = self.quaternion_multiply(orientation, correction_quat)

            # Normalize
            corrected_orientation = corrected_orientation / np.linalg.norm(corrected_orientation)
        else:
            corrected_orientation = orientation

        return corrected_orientation

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])

    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to 3x3 rotation matrix"""
        w, x, y, z = q

        # Rotation matrix from quaternion
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])

        return R

    def publish_orientation(self, header):
        """Publish estimated orientation"""
        imu_msg = Imu()
        imu_msg.header = header

        # Set estimated orientation
        imu_msg.orientation = Quaternion(*self.orientation)
        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Also set angular velocity and linear acceleration
        imu_msg.angular_velocity = Vector3(0.0, 0.0, 0.0)
        imu_msg.linear_acceleration = Vector3(0.0, 0.0, 0.0)

        self.orientation_pub.publish(imu_msg)

    def run(self):
        """Main estimation loop"""
        rospy.loginfo("Orientation estimator started")
        rospy.spin()

if __name__ == '__main__':
    estimator = OrientationEstimator()
    estimator.run()
```

## IMU-Based State Estimation

### Extended Kalman Filter for State Estimation

```python
#!/usr/bin/env python3
"""
Extended Kalman Filter for IMU-based state estimation
"""

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class IMUEKF:
    """Extended Kalman Filter for IMU-based state estimation"""

    def __init__(self):
        rospy.init_node('imu_ekf')

        # Publishers
        self.state_pub = rospy.Publisher('/robot/state_estimate', PoseWithCovarianceStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/robot/odom_imu', Odometry, queue_size=10)

        # Subscribers
        self.imu_sub = rospy.Subscriber('/robot/imu/data', Imu, self.imu_callback)

        # State vector: [x, y, z, vx, vy, vz, qw, qx, qy, qz, bgx, bgy, bgz]
        # Position, Velocity, Orientation, Gyro bias
        self.state_dim = 13
        self.state = np.zeros(self.state_dim)

        # Initial state
        self.state[6] = 1.0  # Initial orientation (w=1, x=y=z=0)

        # Covariance matrix
        self.P = np.eye(self.state_dim) * 0.1

        # Process noise
        self.Q = np.eye(self.state_dim)
        self.Q[:3, :3] *= 0.1    # Position process noise
        self.Q[3:6, 3:6] *= 0.5  # Velocity process noise
        self.Q[6:10, 6:10] *= 0.01  # Orientation process noise
        self.Q[10:, 10:] *= 0.001  # Bias process noise

        # Measurement noise
        self.R_imu = np.eye(6)  # Angular velocity and linear acceleration
        self.R_imu[:3, :3] *= 0.01  # Angular velocity noise
        self.R_imu[3:, 3:] *= 0.05  # Linear acceleration noise

        # Gravity vector
        self.gravity = np.array([0, 0, 9.81])

        # Timing
        self.last_time = rospy.Time.now()

    def imu_callback(self, imu_msg):
        """Process IMU measurements and update state estimate"""
        # Extract measurements
        omega = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])

        linear_acc = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])

        # Get time delta
        current_time = imu_msg.header.stamp
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        if dt <= 0:
            return

        # Prediction step
        self.predict_state(omega, linear_acc, dt)

        # Update step
        measurement = np.concatenate([omega, linear_acc])
        self.update_state(measurement)

        # Publish state estimate
        self.publish_state_estimate(imu_msg.header)

    def predict_state(self, omega, linear_acc, dt):
        """Prediction step of the EKF"""
        # Extract state components
        pos = self.state[:3]
        vel = self.state[3:6]
        quat = self.state[6:10]
        gyro_bias = self.state[10:]

        # Corrected angular velocity
        omega_corrected = omega - gyro_bias

        # State transition Jacobian (linearized around current state)
        F = self.compute_jacobian_F(omega_corrected, dt)

        # Predict state
        self.state = self.state_transition(self.state, omega_corrected, linear_acc, dt)

        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q * dt

    def state_transition(self, state, omega, linear_acc, dt):
        """Nonlinear state transition function"""
        new_state = state.copy()

        # Extract components
        pos = state[:3]
        vel = state[3:6]
        quat = state[6:10]
        gyro_bias = state[10:]

        # Update position: p_new = p + v*dt + 0.5*a*dt^2
        rotation_matrix = self.quaternion_to_rotation_matrix(quat)
        world_linear_acc = rotation_matrix @ (linear_acc - self.gravity)
        new_state[:3] = pos + vel * dt + 0.5 * world_linear_acc * dt**2

        # Update velocity: v_new = v + a*dt
        new_state[3:6] = vel + world_linear_acc * dt

        # Update orientation using quaternion integration
        omega_quat = np.concatenate([omega, [0.0]])
        quat_derivative = 0.5 * self.quaternion_multiply(quat, omega_quat)
        new_quat = quat + quat_derivative * dt
        new_quat = new_quat / np.linalg.norm(new_quat)
        new_state[6:10] = new_quat

        # Gyro bias is modeled as random walk
        # No change in bias prediction (could add drift model)

        return new_state

    def compute_jacobian_F(self, omega, dt):
        """Compute the Jacobian of the state transition function"""
        F = np.eye(self.state_dim)

        # Position-velocity relationship
        F[0:3, 3:6] = np.eye(3) * dt

        # Velocity-acceleration relationship (depends on orientation)
        # This is a simplified version - in practice, this would be more complex
        F[3:6, 6:10] = self.compute_velocity_quat_jacobian(dt)

        # Orientation-angular velocity relationship
        omega_skew = np.array([
            [0, -omega[2], omega[1]],
            [omega[2], 0, -omega[0]],
            [-omega[1], omega[0], 0]
        ])
        F[6:9, 6:9] = np.eye(3) + omega_skew * dt  # Simplified quaternion derivative

        return F

    def compute_velocity_quat_jacobian(self, dt):
        """Compute Jacobian of velocity w.r.t. orientation"""
        # Simplified - in practice, this would involve the rotation matrix derivative
        return np.zeros((3, 4))  # Placeholder

    def update_state(self, measurement):
        """Update step of the EKF"""
        # Measurement model: extract angular velocity and linear acceleration from state
        # In this simplified model, we directly use the measurements
        # A more sophisticated model would predict measurements from state

        # Innovation (measurement residual)
        innovation = measurement - self.predict_measurement()

        # Innovation covariance
        H = self.compute_jacobian_H()
        S = H @ self.P @ H.T + self.R_imu

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ innovation

        # Update covariance
        I = np.eye(self.state_dim)
        self.P = (I - K @ H) @ self.P

        # Normalize quaternion part of state
        quat = self.state[6:10]
        quat = quat / np.linalg.norm(quat)
        self.state[6:10] = quat

    def predict_measurement(self):
        """Predict measurement from current state"""
        # In this simplified model, we return the most recent measurement
        # A full implementation would predict measurements from state
        return np.zeros(6)  # Placeholder

    def compute_jacobian_H(self):
        """Compute Jacobian of measurement function"""
        H = np.zeros((6, self.state_dim))
        H[:3, 10:13] = -np.eye(3)  # Angular velocity depends on bias
        # Other elements depend on specific measurement model
        return H

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])

    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to 3x3 rotation matrix"""
        w, x, y, z = q

        # Rotation matrix from quaternion
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])

        return R

    def publish_state_estimate(self, header):
        """Publish state estimate as PoseWithCovarianceStamped"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = 'map'

        # Set position
        pose_msg.pose.pose.position.x = self.state[0]
        pose_msg.pose.pose.position.y = self.state[1]
        pose_msg.pose.pose.position.z = self.state[2]

        # Set orientation
        pose_msg.pose.pose.orientation.w = self.state[6]
        pose_msg.pose.pose.orientation.x = self.state[7]
        pose_msg.pose.pose.orientation.y = self.state[8]
        pose_msg.pose.pose.orientation.z = self.state[9]

        # Set covariance (flatten 13x13 to 6x6 for position/orientation)
        pose_msg.pose.covariance = np.zeros(36)
        # Position covariance
        pose_msg.pose.covariance[0] = self.P[0, 0]  # xx
        pose_msg.pose.covariance[7] = self.P[1, 1]  # yy
        pose_msg.pose.covariance[14] = self.P[2, 2]  # zz
        # Orientation covariance (indices 21, 28, 35 for xx, yy, zz)
        pose_msg.pose.covariance[21] = self.P[6, 6]  # oxox
        pose_msg.pose.covariance[28] = self.P[7, 7]  # oyoy
        pose_msg.pose.covariance[35] = self.P[8, 8]  # ozoz

        self.state_pub.publish(pose_msg)

        # Also publish as odometry
        odom_msg = Odometry()
        odom_msg.header = header
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose = pose_msg.pose  # Copy pose data

        # Set velocity from state
        odom_msg.twist.twist.linear.x = self.state[3]
        odom_msg.twist.twist.linear.y = self.state[4]
        odom_msg.twist.twist.linear.z = self.state[5]

        # Angular velocity from IMU (corrected for bias)
        # For now, set to zero - would come from state in full implementation
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom_msg)

    def run(self):
        """Main EKF loop"""
        rospy.loginfo("IMU EKF started")
        rospy.spin()

if __name__ == '__main__':
    ekf = IMUEKF()
    ekf.run()
```

## IMU Validation

### Accuracy Assessment and Calibration

```python
#!/usr/bin/env python3
"""
IMU validation for digital twin systems
"""

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

class IMUValidator:
    """Validates IMU simulation accuracy against ground truth"""

    def __init__(self):
        rospy.init_node('imu_validator')

        # Publishers for validation metrics
        self.angular_accuracy_pub = rospy.Publisher('/imu_validation/angular_accuracy', Float64, queue_size=10)
        self.linear_accuracy_pub = rospy.Publisher('/imu_validation/linear_accuracy', Float64, queue_size=10)
        self.orientation_accuracy_pub = rospy.Publisher('/imu_validation/orientation_accuracy', Float64, queue_size=10)

        # Subscribers
        self.sim_imu_sub = rospy.Subscriber('/robot/imu/data', Imu, self.simulated_imu_callback)
        self.gt_imu_sub = rospy.Subscriber('/ground_truth/imu/data', Imu, self.ground_truth_imu_callback)

        # Validation parameters
        self.angular_tolerance = 0.017  # 1 degree in radians
        self.linear_tolerance = 0.05   # 5 cm/s²
        self.orientation_tolerance = 0.017  # 1 degree in radians
        self.validation_window = 100   # Number of samples to average

        # Storage
        self.sim_imu_data = None
        self.gt_imu_data = None
        self.angular_errors = []
        self.linear_errors = []
        self.orientation_errors = []

    def simulated_imu_callback(self, sim_msg):
        """Process simulated IMU data"""
        self.sim_imu_data = sim_msg

        # If we have ground truth, validate
        if self.gt_imu_data is not None:
            self.validate_imu_data()

    def ground_truth_imu_callback(self, gt_msg):
        """Process ground truth IMU data"""
        self.gt_imu_data = gt_msg

    def validate_imu_data(self):
        """Compare simulated IMU data with ground truth"""
        if self.sim_imu_data is None or self.gt_imu_data is None:
            return

        # Calculate angular velocity errors
        sim_angular = np.array([
            self.sim_imu_data.angular_velocity.x,
            self.sim_imu_data.angular_velocity.y,
            self.sim_imu_data.angular_velocity.z
        ])

        gt_angular = np.array([
            self.gt_imu_data.angular_velocity.x,
            self.gt_imu_data.angular_velocity.y,
            self.gt_imu_data.angular_velocity.z
        ])

        angular_error = np.linalg.norm(sim_angular - gt_angular)
        self.angular_errors.append(angular_error)

        # Calculate linear acceleration errors
        sim_linear = np.array([
            self.sim_imu_data.linear_acceleration.x,
            self.sim_imu_data.linear_acceleration.y,
            self.sim_imu_data.linear_acceleration.z
        ])

        gt_linear = np.array([
            self.gt_imu_data.linear_acceleration.x,
            self.gt_imu_data.linear_acceleration.y,
            self.gt_imu_data.linear_acceleration.z
        ])

        linear_error = np.linalg.norm(sim_linear - gt_linear)
        self.linear_errors.append(linear_error)

        # Calculate orientation errors
        sim_orient = np.array([
            self.sim_imu_data.orientation.x,
            self.sim_imu_data.orientation.y,
            self.sim_imu_data.orientation.z,
            self.sim_imu_data.orientation.w
        ])

        gt_orient = np.array([
            self.gt_imu_data.orientation.x,
            self.gt_imu_data.orientation.y,
            self.gt_imu_data.orientation.z,
            self.gt_imu_data.orientation.w
        ])

        # Calculate quaternion distance
        dot_product = np.abs(np.dot(sim_orient, gt_orient))
        orientation_error = 2 * np.arccos(min(1.0, dot_product))  # Angle between quaternions
        self.orientation_errors.append(orientation_error)

        # Keep only recent history
        if len(self.angular_errors) > self.validation_window:
            self.angular_errors.pop(0)
        if len(self.linear_errors) > self.validation_window:
            self.linear_errors.pop(0)
        if len(self.orientation_errors) > self.validation_window:
            self.orientation_errors.pop(0)

        # Calculate and publish accuracy metrics
        if len(self.angular_errors) > 0:
            avg_angular_error = np.mean(self.angular_errors)
            angular_accuracy = max(0.0, 1.0 - avg_angular_error / self.angular_tolerance)
            self.angular_accuracy_pub.publish(Float64(angular_accuracy))

        if len(self.linear_errors) > 0:
            avg_linear_error = np.mean(self.linear_errors)
            linear_accuracy = max(0.0, 1.0 - avg_linear_error / self.linear_tolerance)
            self.linear_accuracy_pub.publish(Float64(linear_accuracy))

        if len(self.orientation_errors) > 0:
            avg_orientation_error = np.mean(self.orientation_errors)
            orientation_accuracy = max(0.0, 1.0 - avg_orientation_error / self.orientation_tolerance)
            self.orientation_accuracy_pub.publish(Float64(orientation_accuracy))

        # Log validation results periodically
        if len(self.angular_errors) % 10 == 0:
            self.log_validation_results()

    def log_validation_results(self):
        """Log validation statistics"""
        if not self.angular_errors:
            return

        # Calculate statistics for recent data
        recent_angular = self.angular_errors[-10:]
        recent_linear = self.linear_errors[-10:]
        recent_orientation = self.orientation_errors[-10:]

        avg_angular = np.mean(recent_angular)
        avg_linear = np.mean(recent_linear)
        avg_orientation = np.mean(recent_orientation)

        # Calculate accuracy percentages
        angular_valid = sum(1 for e in recent_angular if e <= self.angular_tolerance)
        linear_valid = sum(1 for e in recent_linear if e <= self.linear_tolerance)
        orientation_valid = sum(1 for e in recent_orientation if e <= self.orientation_tolerance)

        angular_accuracy_pct = (angular_valid / len(recent_angular)) * 100
        linear_accuracy_pct = (linear_valid / len(recent_linear)) * 100
        orientation_accuracy_pct = (orientation_valid / len(recent_orientation)) * 100

        rospy.loginfo(
            f"IMU Validation - "
            f"Angular: {angular_accuracy_pct:.1f}% ({avg_angular:.3f} rad/s), "
            f"Linear: {linear_accuracy_pct:.1f}% ({avg_linear:.3f} m/s²), "
            f"Orientation: {orientation_accuracy_pct:.1f}% ({avg_orientation:.3f} rad)"
        )

        # Check for validation failures
        if angular_accuracy_pct < 90 or linear_accuracy_pct < 90 or orientation_accuracy_pct < 90:
            rospy.logwarn(
                f"IMU accuracy degraded: "
                f"Angular: {angular_accuracy_pct:.1f}%, "
                f"Linear: {linear_accuracy_pct:.1f}%, "
                f"Orientation: {orientation_accuracy_pct:.1f}%"
            )

    def generate_validation_report(self):
        """Generate comprehensive validation report"""
        if not self.angular_errors:
            rospy.loginfo("No validation data available")
            return

        # Calculate overall statistics
        avg_angular_error = np.mean(self.angular_errors)
        avg_linear_error = np.mean(self.linear_errors)
        avg_orientation_error = np.mean(self.orientation_errors)

        std_angular_error = np.std(self.angular_errors)
        std_linear_error = np.std(self.linear_errors)
        std_orientation_error = np.std(self.orientation_errors)

        total_samples = len(self.angular_errors)
        angular_valid = sum(1 for e in self.angular_errors if e <= self.angular_tolerance)
        linear_valid = sum(1 for e in self.linear_errors if e <= self.linear_tolerance)
        orientation_valid = sum(1 for e in self.orientation_errors if e <= self.orientation_tolerance)

        angular_accuracy = (angular_valid / total_samples) * 100
        linear_accuracy = (linear_valid / total_samples) * 100
        orientation_accuracy = (orientation_valid / total_samples) * 100

        rospy.loginfo("\n=== IMU Simulation Validation Report ===")
        rospy.loginfo(f"Total samples: {total_samples}")
        rospy.loginfo(f"Angular velocity accuracy: {angular_accuracy:.1f}%")
        rospy.loginfo(f"  - Average error: {avg_angular_error:.3f} rad/s ± {std_angular_error:.3f}")
        rospy.loginfo(f"  - Tolerance: {self.angular_tolerance:.3f} rad/s")

        rospy.loginfo(f"Linear acceleration accuracy: {linear_accuracy:.1f}%")
        rospy.loginfo(f"  - Average error: {avg_linear_error:.3f} m/s² ± {std_linear_error:.3f}")
        rospy.loginfo(f"  - Tolerance: {self.linear_tolerance:.3f} m/s²")

        rospy.loginfo(f"Orientation accuracy: {orientation_accuracy:.1f}%")
        rospy.loginfo(f"  - Average error: {avg_orientation_error:.3f} rad ± {std_orientation_error:.3f}")
        rospy.loginfo(f"  - Tolerance: {self.orientation_tolerance:.3f} rad")
        rospy.loginfo("========================================")

        # Return validation summary
        return {
            'angular_accuracy': angular_accuracy,
            'linear_accuracy': linear_accuracy,
            'orientation_accuracy': orientation_accuracy,
            'avg_angular_error': avg_angular_error,
            'avg_linear_error': avg_linear_error,
            'avg_orientation_error': avg_orientation_error,
            'is_valid': (angular_accuracy >= 90 and
                        linear_accuracy >= 90 and
                        orientation_accuracy >= 90)
        }

    def run(self):
        """Main validation loop"""
        rate = rospy.Rate(1)  # Log once per second

        while not rospy.is_shutdown():
            rate.sleep()

        # Generate final report on shutdown
        self.generate_validation_report()

if __name__ == '__main__':
    validator = IMUValidator()
    rospy.loginfo("IMU validator started")
    validator.run()
```

## Summary

IMU simulation is essential for creating accurate digital twins of humanoid robots. Proper configuration with realistic noise models, bias characteristics, and validation ensures that the digital twin accurately represents the physical robot's inertial sensing capabilities. The implementation of sensor fusion algorithms, orientation estimation, and state tracking enables effective training and validation of balance control and navigation systems before deployment on physical robots.

## Next Steps

Continue to learn about implementing learning objectives and outcomes for the sensor simulation chapter.

[← Previous: Depth Camera Simulation](./depth-camera-simulation) | [Next: Sensor Learning Objectives →](./sensor-learning-objectives)