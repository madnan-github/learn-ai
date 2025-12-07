---
sidebar_position: 3
title: 'Creating Robot Models in Gazebo'
---

# Creating Robot Models in Gazebo

This chapter focuses on adapting your URDF models for Gazebo simulation, including adding Gazebo-specific plugins, sensors, and physical properties to create realistic robot simulations.

## What You'll Learn

In this chapter, you'll explore:
- Adapting URDF models for Gazebo simulation
- Adding Gazebo plugins for robot control
- Configuring sensors for simulation
- Setting up joint controllers
- Testing robot models in Gazebo environments

## Prerequisites

- Completion of Module 1 (especially URDF chapter)
- Completion of Module 2, Chapter 2
- Gazebo Harmonic installed
- Basic understanding of robot kinematics

## Adapting URDF for Gazebo

### Gazebo-Specific Tags

To make your URDF work properly in Gazebo, you need to add Gazebo-specific tags:

```xml
<?xml version="1.0"?>
<robot name="gazebo_humanoid">

  <!-- Links and joints as in URDF -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Gazebo-specific material definition -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

</robot>
```

### Gazebo Link Properties

You can specify additional properties for links in Gazebo:

```xml
<gazebo reference="link_name">
  <mu1>0.2</mu1>                    <!-- Friction coefficient 1 -->
  <mu2>0.2</mu2>                    <!-- Friction coefficient 2 -->
  <kp>1000000.0</kp>                <!-- Spring stiffness -->
  <kd>100.0</kd>                    <!-- Damping coefficient -->
  <material>Gazebo/Blue</material>  <!-- Visual material -->
  <self_collide>false</self_collide> <!-- Enable self-collision -->
  <gravity>true</gravity>            <!-- Enable gravity for this link -->
  <max_contacts>10</max_contacts>    <!-- Max contacts with other objects -->
</gazebo>
```

## Adding Gazebo Plugins

### Joint State Publisher Plugin

To publish joint states from Gazebo:

```xml
<gazebo>
  <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
    <joint_name>left_hip</joint_name>
    <joint_name>left_knee</joint_name>
    <joint_name>right_hip</joint_name>
    <joint_name>right_knee</joint_name>
    <!-- Add more joint names as needed -->
  </plugin>
</gazebo>
```

### Diff Drive Controller Plugin

For wheeled robots:

```xml
<gazebo>
  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_radius>0.1</wheel_radius>
    <odom_publish_frequency>30</odom_publish_frequency>
    <topic>cmd_vel</topic>
    <odom_topic>odom</odom_topic>
    <tf_topic>tf</tf_topic>
  </plugin>
</gazebo>
```

### Joint Position Controller Plugin

For humanoid robots with revolute joints:

```xml
<gazebo>
  <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
    <joint_name>left_hip</joint_name>
    <topic>left_hip_position_controller/command</topic>
    <update_rate>100</update_rate>
  </plugin>
</gazebo>
```

## Adding Sensors to Robots

### IMU Sensor

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
```

### Camera Sensor

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
  <sensor name="camera_sensor" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
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
    </camera>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <topic>camera/image_raw</topic>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR Sensor

```xml>
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

<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="gpu_lidar">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
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
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <topic>scan</topic>
    </plugin>
  </sensor>
</gazebo>
```

## Complete Humanoid Robot Model for Gazebo

Here's a complete humanoid robot model with Gazebo integration:

```xml
<?xml version="1.0"?>
<robot name="gazebo_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

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

  <!-- Gazebo properties for base link -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

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
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <!-- Left leg -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2"/>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2"/>
    </collision>

    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_hip" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="0 0.1 -0.05"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2"/>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2"/>
    </collision>

    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
    <dynamics damping="0.3" friction="0.05"/>
  </joint>

  <!-- Right leg -->
  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2"/>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2"/>
    </collision>

    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_hip" type="revolute">
    <parent link="base_link"/>
    <child link="right_thigh"/>
    <origin xyz="0 -0.1 -0.05"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <link name="right_shin">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2"/>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2"/>
    </collision>

    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_knee" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.4"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
    <dynamics damping="0.3" friction="0.05"/>
  </joint>

  <!-- Left arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.15"/>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.15"/>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.0075" ixy="0.0" ixz="0.0" iyy="0.0075" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.2"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
    <dynamics damping="0.2" friction="0.02"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.15"/>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.15"/>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.006" ixy="0.0" ixz="0.0" iyy="0.006" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="50" velocity="1"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <!-- Right arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.15"/>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.15"/>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.0075" ixy="0.0" ixz="0.0" iyy="0.0075" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.2"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
    <dynamics damping="0.2" friction="0.02"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.15"/>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 -0.15"/>
    </collision>

    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.006" ixy="0.0" ixz="0.0" iyy="0.006" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="50" velocity="1"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo>
    <!-- Joint state publisher -->
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <joint_name>left_hip</joint_name>
      <joint_name>left_knee</joint_name>
      <joint_name>right_hip</joint_name>
      <joint_name>right_knee</joint_name>
      <joint_name>left_shoulder</joint_name>
      <joint_name>left_elbow</joint_name>
      <joint_name>right_shoulder</joint_name>
      <joint_name>right_elbow</joint_name>
      <joint_name>neck_joint</joint_name>
    </plugin>

    <!-- IMU sensor -->
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <topic>imu/data</topic>
      <visualize>true</visualize>
    </sensor>
  </gazebo>

  <!-- Gazebo properties for other links -->
  <gazebo reference="torso">
    <material>Gazebo/White</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/Yellow</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="left_thigh">
    <material>Gazebo/Gray</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="left_shin">
    <material>Gazebo/Gray</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="right_thigh">
    <material>Gazebo/Gray</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="right_shin">
    <material>Gazebo/Gray</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="left_upper_arm">
    <material>Gazebo/Gray</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="left_lower_arm">
    <material>Gazebo/Gray</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="right_upper_arm">
    <material>Gazebo/Gray</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="right_lower_arm">
    <material>Gazebo/Gray</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

</robot>
```

## Hands-on Lab: Creating and Testing Your Robot Model

In this lab, you'll create a complete humanoid robot model and test it in Gazebo.

### Step 1: Create the Model Directory Structure

```bash
# Create model directory
mkdir -p ~/.gazebo/models/gazebo_humanoid

# Create the model.config file
cat > ~/.gazebo/models/gazebo_humanoid/model.config << EOF
<?xml version="1.0"?>
<model>
  <name>gazebo_humanoid</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>
  <description>A simple humanoid robot for Gazebo simulation.</description>
</model>
EOF
```

### Step 2: Convert URDF to SDF

Gazebo works with SDF format, so you'll need to convert your URDF. Create the SDF file:

```bash
# Create the model.sdf file from your URDF
# You can use the URDF from the previous example and convert it to SDF
```

### Step 3: Create a Launch File

Create `launch_humanoid.launch.py`:

```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the launch directory
    gazebo_ros_launch_dir = PathJoinSubstitution(
        [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_ros_launch_dir),
        launch_arguments={'verbose': 'false'}.items(),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid_robot',
            '-x', '0', '-y', '0', '-z', '1.0',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True}],
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True}],
    )

    ld = LaunchDescription()

    # Add the actions
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)

    return ld
```

### Step 4: Test the Robot

1. Launch the simulation:
```bash
ros2 launch launch_humanoid.launch.py
```

2. In another terminal, check if the robot is properly loaded:
```bash
# Check topics
ros2 topic list | grep -E "(joint|imu|scan)"

# Check TF tree
ros2 run tf2_tools view_frames
```

## Troubleshooting Robot Models

### Common Issues and Solutions

1. **Robot falls through the ground**: Check mass and inertia values
2. **Joints are unstable**: Increase damping or reduce joint limits
3. **Robot doesn't respond to commands**: Verify plugin configuration
4. **Sensors don't publish data**: Check sensor configuration and topics

### Debugging Commands

```bash
# Check robot description
ros2 param get /robot_state_publisher robot_description

# Check joint states
ros2 topic echo /joint_states

# Visualize in RViz
rviz2
```

## Best Practices

1. **Realistic Physics**: Use realistic mass and inertia values
2. **Proper Damping**: Add appropriate damping to prevent oscillations
3. **Joint Limits**: Set realistic joint limits based on real robot capabilities
4. **Sensors**: Include appropriate sensors for your application
5. **Testing**: Test models in simple environments before complex ones
6. **Performance**: Balance detail with simulation performance

## Next Steps

After completing this chapter, you'll be ready to learn about sensor simulation in Chapter 4, where you'll explore how to properly simulate LiDAR, cameras, and IMUs for humanoid robots.