---
sidebar_position: 4
title: 'Sensor Simulation'
---

# Sensor Simulation

This chapter covers the simulation of various sensors in Gazebo, including LiDAR, cameras, depth sensors, and IMUs, which are essential for humanoid robot perception and navigation.

## What You'll Learn

In this chapter, you'll explore:
- LiDAR simulation for environment mapping
- Camera and depth camera simulation
- IMU and inertial sensor simulation
- Sensor fusion concepts in simulation
- Troubleshooting sensor data issues

## Prerequisites

- Completion of Module 1 and 2, Chapters 1-3
- Gazebo Harmonic installed
- Basic understanding of sensor principles
- ROS 2 Humble installed

## LiDAR Simulation

### Understanding LiDAR in Simulation

LiDAR (Light Detection and Ranging) sensors provide 2D or 3D distance measurements. In Gazebo, LiDAR sensors can be either CPU-based or GPU-based:

- **CPU-based**: More accurate but slower
- **GPU-based**: Faster but may have limitations with transparency

### 2D LiDAR Configuration

```xml
<link name="laser_link">
  <inertial>
    <mass value="0.1"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
  <visual>
    <geometry>
      <cylinder length="0.05" radius="0.02"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.05" radius="0.02"/>
    </geometry>
  </collision>
</link>

<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser_link"/>
  <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
</joint>

<gazebo reference="laser_link">
  <sensor name="laser_scan" type="gpu_lidar">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <visualize>false</visualize>
    <topic>scan</topic>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle> <!-- -90 degrees -->
          <max_angle>1.570796</max_angle>   <!-- 90 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </sensor>
</gazebo>
```

### 3D LiDAR Configuration

```xml
<gazebo reference="lidar_3d_link">
  <sensor name="lidar_3d" type="gpu_lidar">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <visualize>false</visualize>
    <topic>points2</topic>
    <ray>
      <scan>
        <horizontal>
          <samples>640</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle> <!-- -180 degrees -->
          <max_angle>3.14159</max_angle>   <!-- 180 degrees -->
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.2618</min_angle> <!-- -15 degrees -->
          <max_angle>0.2618</max_angle>   <!-- 15 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>100.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>
    </noise>
  </sensor>
</gazebo>
```

## Camera Simulation

### RGB Camera Configuration

```xml
<link name="camera_link">
  <inertial>
    <mass value="0.01"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.02 0.04 0.02"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.02 0.04 0.02"/>
    </geometry>
  </collision>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    <topic>camera/image_raw</topic>
    <camera>
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees in radians -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
  </sensor>
</gazebo>
```

### Depth Camera Configuration

```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth_camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    <topic>camera/depth/image_raw</topic>
    <camera>
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees in radians -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <output_type>depths</output_type>
  </sensor>
</gazebo>
```

### Stereo Camera Configuration

```xml
<!-- Left camera -->
<link name="left_camera_link">
  <inertial>
    <mass value="0.01"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.01 0.02 0.01"/>
    </geometry>
  </visual>
</link>

<joint name="left_camera_joint" type="fixed">
  <parent link="head"/>
  <child link="left_camera_link"/>
  <origin xyz="0.06 0.05 0" rpy="0 0 0"/>
</joint>

<!-- Right camera -->
<link name="right_camera_link">
  <inertial>
    <mass value="0.01"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.01 0.02 0.01"/>
    </geometry>
  </visual>
</link>

<joint name="right_camera_joint" type="fixed">
  <parent link="head"/>
  <child link="right_camera_link"/>
  <origin xyz="0.06 -0.05 0" rpy="0 0 0"/>
</joint>

<gazebo reference="left_camera_link">
  <sensor name="left_camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>false</visualize>
    <topic>stereo/left/image_raw</topic>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
  </sensor>
</gazebo>

<gazebo reference="right_camera_link">
  <sensor name="right_camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>false</visualize>
    <topic>stereo/right/image_raw</topic>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

## IMU Simulation

### IMU Sensor Configuration

```xml
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
  </inertial>
  <visual>
    <geometry>
      <box size="0.01 0.01 0.01"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.01 0.01 0.01"/>
    </geometry>
  </collision>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <topic>imu/data</topic>
    <visualize>false</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

## Processing Sensor Data in ROS 2

### LiDAR Data Processing Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np

class LidarProcessor(Node):

    def __init__(self):
        super().__init__('lidar_processor')

        # Subscribe to laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for processed data
        self.obstacle_publisher = self.create_publisher(
            String,
            'obstacle_detection',
            10)

        self.get_logger().info('LiDAR Processor initialized')

    def lidar_callback(self, msg):
        # Convert ranges to numpy array
        ranges = np.array(msg.ranges)

        # Handle invalid readings
        ranges[np.isnan(ranges)] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max

        # Detect obstacles within 1 meter
        obstacle_ranges = ranges[ranges < 1.0]
        obstacle_angle_indices = np.where(ranges < 1.0)[0]

        if len(obstacle_ranges) > 0:
            # Calculate angles for obstacles
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            obstacle_angles = angle_min + obstacle_angle_indices * angle_increment

            # Find closest obstacle
            min_distance_idx = np.argmin(obstacle_ranges)
            closest_distance = obstacle_ranges[min_distance_idx]
            closest_angle = obstacle_angles[min_distance_idx]

            # Publish obstacle information
            obstacle_msg = String()
            obstacle_msg.data = f'Obstacle at {closest_angle:.2f} rad, distance: {closest_distance:.2f}m'
            self.obstacle_publisher.publish(obstacle_msg)

            self.get_logger().info(f'Obstacle detected: {obstacle_msg.data}')
        else:
            self.get_logger().info('No obstacles detected')

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarProcessor()

    try:
        rclpy.spin(lidar_processor)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Camera Data Processing Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraProcessor(Node):

    def __init__(self):
        super().__init__('camera_processor')

        # Create CV bridge to convert ROS images to OpenCV
        self.bridge = CvBridge()

        # Subscribe to camera image
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Camera Processor initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform basic image processing
            # Example: Detect edges using Canny edge detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Draw contours on the original image
            contour_image = cv_image.copy()
            cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)

            # Display the processed image
            cv2.imshow("Camera Feed with Contours", contour_image)
            cv2.waitKey(1)  # Refresh the display

            # Log information about detected contours
            self.get_logger().info(f'Detected {len(contours)} contours')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()

    try:
        rclpy.spin(camera_processor)
    except KeyboardInterrupt:
        pass
    finally:
        camera_processor.destroy_node()
        cv2.destroyAllWindows()  # Close OpenCV windows
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Lab: Multi-Sensor Humanoid Robot

In this lab, you'll create a humanoid robot with multiple sensors and process the sensor data.

### Step 1: Create a Multi-Sensor Robot URDF

Create `multi_sensor_humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="multi_sensor_humanoid">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.15 0.15 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.15 0.15 0.1"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.03"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.2"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- LiDAR sensor -->
  <link name="lidar_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.02"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.02"/>
      </geometry>
    </collision>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.1 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Camera sensor -->
  <link name="camera_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.02 0.04 0.02"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.04 0.02"/>
      </geometry>
    </collision>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
  </joint>

  <!-- IMU sensor -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo plugins and sensors -->
  <gazebo>
    <!-- Joint state publisher -->
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <joint_name>neck_joint</joint_name>
    </plugin>
  </gazebo>

  <!-- LiDAR sensor -->
  <gazebo reference="lidar_link">
    <sensor name="laser_scan" type="gpu_lidar">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <visualize>false</visualize>
      <topic>scan</topic>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </sensor>
  </gazebo>

  <!-- Camera sensor -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <visualize>false</visualize>
      <topic>camera/image_raw</topic>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
    </sensor>
  </gazebo>

  <!-- IMU sensor -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <topic>imu/data</topic>
      <visualize>false</visualize>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>

  <!-- Gazebo materials -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="torso">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/Yellow</material>
  </gazebo>

</robot>
```

### Step 2: Create a Sensor Fusion Node

Create `sensor_fusion_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time

class SensorFusionNode(Node):

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Create CV bridge
        self.bridge = CvBridge()

        # Subscribe to sensors
        self.lidar_subscription = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)
        self.camera_subscription = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, 10)
        self.imu_subscription = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)

        # Publisher for robot commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Store sensor data
        self.lidar_data = None
        self.camera_data = None
        self.imu_data = None

        # Lock for thread safety
        self.data_lock = threading.Lock()

        # Timer for decision making
        self.timer = self.create_timer(0.1, self.fusion_callback)

        self.get_logger().info('Sensor Fusion Node initialized')

    def lidar_callback(self, msg):
        with self.data_lock:
            self.lidar_data = msg

    def camera_callback(self, msg):
        with self.data_lock:
            try:
                self.camera_data = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().error(f'Error converting image: {str(e)}')

    def imu_callback(self, msg):
        with self.data_lock:
            self.imu_data = msg

    def fusion_callback(self):
        with self.data_lock:
            # Check if we have all sensor data
            if self.lidar_data is None or self.imu_data is None:
                return

            # Process LiDAR data for obstacle detection
            ranges = np.array(self.lidar_data.ranges)
            ranges[np.isnan(ranges)] = self.lidar_data.range_max
            ranges[np.isinf(ranges)] = self.lidar_data.range_max

            # Detect obstacles in front
            front_ranges = ranges[len(ranges)//2-30:len(ranges)//2+30]
            min_front_distance = np.min(front_ranges)

            # Process IMU data for orientation
            orientation_z = self.imu_data.orientation.z

            # Create robot command based on sensor fusion
            cmd_vel = Twist()

            if min_front_distance < 0.8:  # Obstacle detected
                # Turn away from obstacle
                cmd_vel.linear.x = 0.2
                cmd_vel.angular.z = 0.5 if orientation_z > 0 else -0.5
                self.get_logger().info(f'Obstacle detected! Distance: {min_front_distance:.2f}m, turning')
            else:
                # Move forward
                cmd_vel.linear.x = 0.5
                cmd_vel.angular.z = 0.0
                self.get_logger().info(f'Clear path, moving forward. Distance: {min_front_distance:.2f}m')

        # Publish the command
        self.cmd_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Test the Multi-Sensor Robot

1. Launch Gazebo with your world:
```bash
gz sim -r your_world.sdf
```

2. Spawn your robot:
```bash
ros2 run gazebo_ros spawn_entity.py -entity multi_sensor_humanoid -file multi_sensor_humanoid.urdf -x 0 -y 0 -z 1
```

3. Run the sensor fusion node:
```bash
python3 sensor_fusion_node.py
```

## Sensor Calibration and Validation

### Validating Sensor Data

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import numpy as np

class SensorValidator(Node):

    def __init__(self):
        super().__init__('sensor_validator')

        self.bridge = CvBridge()

        # Subscribe to all sensors
        self.lidar_subscription = self.create_subscription(
            LaserScan, 'scan', self.validate_lidar, 10)
        self.camera_subscription = self.create_subscription(
            Image, 'camera/image_raw', self.validate_camera, 10)
        self.imu_subscription = self.create_subscription(
            Imu, 'imu/data', self.validate_imu, 10)

    def validate_lidar(self, msg):
        # Check for valid ranges
        valid_ranges = [r for r in msg.ranges if not (np.isnan(r) or np.isinf(r))]

        # Log validation results
        if len(valid_ranges) == 0:
            self.get_logger().warn('LiDAR: No valid ranges detected')
        elif min(valid_ranges) < msg.range_min:
            self.get_logger().warn(f'LiDAR: Range below minimum: {min(valid_ranges)} < {msg.range_min}')
        elif max(valid_ranges) > msg.range_max:
            self.get_logger().warn(f'LiDAR: Range above maximum: {max(valid_ranges)} > {msg.range_max}')
        else:
            self.get_logger().info(f'LiDAR: OK - {len(valid_ranges)} valid ranges, min: {min(valid_ranges):.2f}, max: {max(valid_ranges):.2f}')

    def validate_camera(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width = cv_image.shape[:2]

            if height != msg.height or width != msg.width:
                self.get_logger().warn(f'Camera: Image size mismatch - expected {msg.width}x{msg.height}, got {width}x{height}')
            else:
                self.get_logger().info(f'Camera: OK - {width}x{height} image')
        except Exception as e:
            self.get_logger().error(f'Camera: Error processing image - {str(e)}')

    def validate_imu(self, msg):
        # Check if orientation values are reasonable
        orientation_valid = (
            -1 <= msg.orientation.x <= 1 and
            -1 <= msg.orientation.y <= 1 and
            -1 <= msg.orientation.z <= 1 and
            -1 <= msg.orientation.w <= 1
        )

        if not orientation_valid:
            self.get_logger().warn('IMU: Orientation values out of range')
        else:
            self.get_logger().info('IMU: OK - orientation values valid')

def main(args=None):
    rclpy.init(args=args)
    validator = SensorValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        pass
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

1. **Noise Modeling**: Include realistic noise in sensor simulations
2. **Update Rates**: Match simulation update rates to real sensor capabilities
3. **Visualization**: Use Gazebo's visualization tools to verify sensor behavior
4. **Data Validation**: Always validate sensor data before processing
5. **Sensor Placement**: Place sensors where they would be on a real robot
6. **Performance**: Balance sensor fidelity with simulation performance

## Next Steps

After completing this chapter, you'll be ready to learn about Unity integration for advanced simulation in Chapter 5.