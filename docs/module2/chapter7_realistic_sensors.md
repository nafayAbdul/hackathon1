# Chapter 7: Realistic Sensors in Simulation – Depth, LiDAR, IMU, and Contact

## Learning Objectives
By the end of this chapter, you will be able to:
- Implement realistic sensor models matching Tier 1-4 hardware specifications
- Configure noise models for depth cameras, LiDAR, IMU, and force/torque sensors
- Simultaneously record ground-truth and noisy sensor data
- Compare simulated vs. real sensor outputs for validation

## 7.1 Tier 1-4 Sensor Suite for Athena

In this chapter, we'll implement a comprehensive sensor suite for our Athena humanoid that matches industry-standard Tier 1-4 hardware specifications:

- **Tier 1**: Basic navigation and control sensors
  - IMU (BMI088)
  - Basic contact sensors

- **Tier 2**: Perception capabilities
  - Depth camera (RealSense D455 equivalent)

- **Tier 3**: Advanced perception
  - 64-channel LiDAR
  - High-precision force/torque sensors

- **Tier 4**: Complete sensor fusion
  - All sensors integrated with calibration
  - Simultaneous ground-truth and noisy data recording

## 7.2 RealSense D455 Depth Camera Simulation

The Intel RealSense D455 is the gold standard for RGB-D sensing in robotics. Let's implement a simulation model that matches its specifications:

### Depth Camera SDF Configuration

```xml
<!-- In the Athena URDF/XACRO file -->
<xacro:macro name="realsense_d455_depth_camera" params="prefix parent *origin">
  <!-- Camera link -->
  <link name="${prefix}camera_link">
    <visual>
      <geometry>
        <box size="0.03 0.13 0.03"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.03 0.13 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.065"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <!-- Camera joint -->
  <joint name="${prefix}camera_joint" type="fixed">
    <parent link="${parent}"/>
    <child link="${prefix}camera_link"/>
    <xacro:insert_block name="origin"/>
  </joint>

  <!-- Gazebo plugin for depth camera -->
  <gazebo reference="${prefix}camera_link">
    <sensor name="${prefix}camera" type="depth">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera name="head">
        <horizontal_fov>1.0471975512</horizontal_fov> <!-- 60 degrees -->
        <image>
          <width>1280</width>
          <height>720</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.100</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="gz-sim-rgbd-camera-system">
        <camera_name>${prefix}camera</camera_name>
        <update_rate>30</update_rate>
        <baseline>0.01</baseline>
        <distortion_k1>0.0</distortion_k1>
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
        
        <!-- RealSense D455 specific parameters -->
        <depth_camera>
          <output>depths</output>
        </depth_camera>
      </plugin>
    </sensor>
  </gazebo>
</xacro:macro>
```

## 7.3 64-Channel LiDAR Simulation

The 64-channel LiDAR will be mounted on the head of Athena, providing comprehensive 3D mapping capabilities:

### LiDAR Configuration

```xml
<xacro:macro name="velodyne_vlp64_lidar" params="prefix parent *origin">
  <link name="${prefix}lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.08"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.250"/>
      <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/>
    </inertial>
  </link>

  <joint name="${prefix}lidar_joint" type="fixed">
    <parent link="${parent}"/>
    <child link="${prefix}lidar_link"/>
    <xacro:insert_block name="origin"/>
  </joint>

  <gazebo reference="${prefix}lidar_link">
    <sensor name="${prefix}lidar" type="lidar">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>2250</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
          <vertical>
            <samples>64</samples>
            <resolution>1</resolution>
            <min_angle>-0.261799</min_angle>
            <max_angle>0.261799</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.1</min>
          <max>100.0</max>
          <resolution>0.001</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="gz-sim-lidar-system">
        <topic>${prefix}scan</topic>
        <frame_id>${prefix}lidar_link</frame_id>
        <update_rate>10</update_rate>
      </plugin>
    </sensor>
  </gazebo>
</xacro:macro>
```

## 7.4 BMI088 IMU Simulation

Let's implement the BMI088 IMU with realistic noise characteristics:

### IMU Configuration

```xml
<xacro:macro name="bmi088_imu" params="prefix parent *origin">
  <link name="${prefix}imu_link">
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="${prefix}imu_joint" type="fixed">
    <parent link="${parent}"/>
    <child link="${prefix}imu_link"/>
    <xacro:insert_block name="origin"/>
  </joint>

  <gazebo reference="${prefix}imu_link">
    <sensor name="${prefix}imu" type="imu">
      <always_on>true</always_on>
      <update_rate>400</update_rate>
      <imu>
        <!-- Accelerometer parameters -->
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001351</stddev>
              <bias_mean>0.0</bias_mean>
              <bias_stddev>0.01351</bias_stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001351</stddev>
              <bias_mean>0.0</bias_mean>
              <bias_stddev>0.01351</bias_stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001351</stddev>
              <bias_mean>0.0</bias_mean>
              <bias_stddev>0.01351</bias_stddev>
            </noise>
          </z>
        </linear_acceleration>

        <!-- Gyroscope parameters -->
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.000290888</stddev>
              <bias_mean>0.0</bias_mean>
              <bias_stddev>0.000290888</bias_stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.000290888</stddev>
              <bias_mean>0.0</bias_mean>
              <bias_stddev>0.000290888</bias_stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.000290888</stddev>
              <bias_mean>0.0</bias_mean>
              <bias_stddev>0.000290888</bias_stddev>
            </noise>
          </z>
        </angular_velocity>

        <!-- Magnetometer parameters -->
        <magnetic_field>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>3.1e-7</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>3.1e-7</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>3.1e-7</stddev>
            </noise>
          </z>
        </magnetic_field>
      </imu>
      <plugin name="imu_controller" filename="gz-sim-imu-system">
        <topic>${prefix}imu</topic>
        <frame_id>${prefix}imu_link</frame_id>
        <update_rate>400</update_rate>
      </plugin>
    </sensor>
  </gazebo>
</xacro:macro>
```

## 7.5 Foot Force/Torque Sensors

For humanoid balance and locomotion, we need accurate foot sensors:

### Force/Torque Sensor Configuration

```xml
<xacro:macro name="foot_force_torque_sensor" params="prefix parent *origin">
  <gazebo>
    <plugin name="${prefix}ft_sensor" filename="gz-sim-contact-system">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      
      <!-- Force/Torque sensor parameters -->
      <topic>${prefix}ft_sensor</topic>
      <frame_id>${parent}</frame_id>
      
      <!-- Contact detection -->
      <collision>foot_collision</collision>
      
      <!-- Noise model -->
      <force_noise_mean>0.0</force_noise_mean>
      <force_noise_stddev>1.0</force_noise_stddev>
      <torque_noise_mean>0.0</torque_noise_mean>
      <torque_noise_stddev>0.1</torque_noise_stddev>
    </plugin>
  </gazebo>
</xacro:macro>
```

## 7.6 Noise Models and Realistic Effects

Implementing realistic sensor behavior requires careful modeling of noise characteristics:

### Python Node for Advanced Noise Modeling

```python
#!/usr/bin/env python3
"""
Advanced sensor noise modeling node
Simulates various real-world sensor effects
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu, JointState
from geometry_msgs.msg import Point
import numpy as np
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


class SensorNoiseSimulator(Node):
    def __init__(self):
        super().__init__('sensor_noise_simulator')
        
        # Publishers for noisy data
        self.noisy_image_pub = self.create_publisher(Image, '/athena/camera/image_noisy', 10)
        self.noisy_imu_pub = self.create_publisher(Imu, '/athena/imu/data_noisy', 10)
        self.noisy_lidar_pub = self.create_publisher(PointCloud2, '/athena/lidar/points_noisy', 10)
        
        # Subscribers for ground truth
        self.gt_image_sub = self.create_subscription(Image, '/athena/camera/image_gt', self.image_callback, 10)
        self.gt_imu_sub = self.create_subscription(Imu, '/athena/imu/data_gt', self.imu_callback, 10)
        self.gt_lidar_sub = self.create_subscription(PointCloud2, '/athena/lidar/points_gt', self.lidar_callback, 10)
        
        self.cv_bridge = CvBridge()
        
        # Sensor-specific noise parameters
        self.camera_noise_params = {
            'gaussian_std': 0.05,      # Gaussian noise standard deviation
            'dropout_rate': 0.001,     # Pixel dropout rate
            'rolling_shutter': 0.001,  # Rolling shutter effect
            'temp_drift': 0.0001       # Temperature-related drift
        }
        
        self.imu_noise_params = {
            'acc_bias_std': 0.0001,     # Accelerometer bias standard deviation
            'gyro_bias_std': 0.00001,   # Gyroscope bias standard deviation
            'acc_noise_dens': 0.0019,   # Accelerometer noise density
            'gyro_noise_dens': 0.000014 # Gyroscope noise density
        }
        
        self.lidar_noise_params = {
            'range_noise_std': 0.01,    # Range measurement noise
            'angular_noise_std': 0.001, # Angular measurement noise
            'dropout_rate': 0.0001      # Point dropout rate
        }

    def image_callback(self, msg):
        """Apply noise to camera image"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Apply Gaussian noise
            gaussian_noise = np.random.normal(0, self.camera_noise_params['gaussian_std'], cv_image.shape)
            noisy_image = cv_image + gaussian_noise
            
            # Apply pixel dropout
            dropout_mask = np.random.rand(*cv_image.shape[:2]) < self.camera_noise_params['dropout_rate']
            noisy_image[dropout_mask] = 0
            
            # Convert back to ROS message
            noisy_msg = self.cv_bridge.cv2_to_imgmsg(noisy_image.astype(np.uint8), encoding=msg.encoding)
            noisy_msg.header = msg.header
            self.noisy_image_pub.publish(noisy_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def imu_callback(self, msg):
        """Apply noise to IMU data"""
        noisy_msg = Imu()
        noisy_msg.header = msg.header
        
        # Apply noise to linear acceleration
        acc_noise = np.random.normal(0, self.imu_noise_params['acc_noise_dens'], 3)
        noisy_msg.linear_acceleration.x = msg.linear_acceleration.x + acc_noise[0]
        noisy_msg.linear_acceleration.y = msg.linear_acceleration.y + acc_noise[1]
        noisy_msg.linear_acceleration.z = msg.linear_acceleration.z + acc_noise[2]
        
        # Apply noise to angular velocity
        gyro_noise = np.random.normal(0, self.imu_noise_params['gyro_noise_dens'], 3)
        noisy_msg.angular_velocity.x = msg.angular_velocity.x + gyro_noise[0]
        noisy_msg.angular_velocity.y = msg.angular_velocity.y + gyro_noise[1]
        noisy_msg.angular_velocity.z = msg.angular_velocity.z + gyro_noise[2]
        
        # Copy orientation (assuming perfect orientation for simplicity)
        noisy_msg.orientation = msg.orientation
        
        self.noisy_imu_pub.publish(noisy_msg)

    def lidar_callback(self, msg):
        """Apply noise to LiDAR point cloud"""
        points_list = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            # Apply range noise
            noise = np.random.normal(0, self.lidar_noise_params['range_noise_std'], 3)
            
            # Apply noise to point
            noisy_point = Point()
            noisy_point.x = point[0] + noise[0]
            noisy_point.y = point[1] + noise[1]
            noisy_point.z = point[2] + noise[2]
            
            points_list.append(noisy_point)
        
        # Convert back to PointCloud2
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id
        noisy_cloud = pc2.create_cloud(header, msg.fields, points_list)
        
        self.noisy_lidar_pub.publish(noisy_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = SensorNoiseSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 7.7 Side-by-Side Comparisons: Simulated vs. Real

To validate our sensor simulations, implement a comparison tool that displays simulated and real data simultaneously. The comparison would show:

- **Depth Camera**: Simulated point clouds vs. real RealSense point clouds
- **LiDAR**: 64-channel simulated returns vs. real LiDAR data
- **IMU**: Simulated orientation, acceleration, and angular velocity vs. real sensor readings
- **Force/Torque**: Simulated contact forces vs. real sensor data

## 7.8 Recording Ground-Truth and Noisy Data

Implement a data recording system that captures both ground-truth and noisy sensor data:

### Data Recording Launch File

```xml
<!-- launch/sensor_data_recording.launch.xml -->
<launch>
  <!-- Start the robot and sensors -->
  <include file="$(find-pkg-share athena_description)/launch/spawn_athena.launch.py"/>
  
  <!-- Start the noise simulator -->
  <node pkg="athena_examples" exec="sensor_noise_simulator.py" name="sensor_noise_simulator"/>
  
  <!-- Start rosbags for ground-truth data -->
  <node pkg="ros2bag" exec="record" name="gt_recorder" 
        args="-o /data/sensor_gt --include-topics /athena/camera/image_gt /athena/imu/data_gt /athena/lidar/points_gt"/>
  
  <!-- Start rosbags for noisy data -->
  <node pkg="ros2bag" exec="record" name="noisy_recorder" 
        args="-o /data/sensor_noisy --include-topics /athena/camera/image_noisy /athena/imu/data_noisy /athena/lidar/points_noisy"/>
  
  <!-- Visualization tools -->
  <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share athena_examples)/rviz/sensor_comparison.rviz"/>
</launch>
```

## "Pro Tips" Sidebar

- **Sensor Fusion**: When implementing multiple sensors, consider the timing synchronization between different sensor modalities.
- **Calibration**: Simulated sensors should be easily calibrated to match real sensors for sim-to-real transfer.
- **Validation**: Always compare your simulated sensor data with real sensor data to validate realism.

## References to Official Documentation

- [Gazebo Sensor Documentation](https://gazebosim.org/docs/harmonic/sensors/)
- [RealSense D455 Specifications](https://www.intel.com/content/www/us/en/products/docs/movidius-cameras/d455-product-brief.html)
- [ROS 2 Sensor Message Types](https://docs.ros.org/en/rolling/)

In the next chapter, we'll explore photorealistic rendering techniques using Unity and Unreal Engine for enhanced human-robot interaction.