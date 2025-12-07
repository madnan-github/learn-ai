---
sidebar_position: 2
title: 'Gazebo Physics Simulation'
---

# Gazebo Physics Simulation

This chapter covers the fundamentals of physics simulation in Gazebo, including how to create realistic environments for humanoid robots with proper physics properties.

## What You'll Learn

In this chapter, you'll explore:
- Gazebo simulation environment setup
- Physics engine concepts and parameters
- Creating realistic physics environments
- Configuring gravity, friction, and damping
- Debugging physics simulation issues

## Prerequisites

- Completion of Module 1
- ROS 2 Humble installed
- Gazebo Harmonic installed
- Basic understanding of physics concepts

## Understanding Gazebo Physics

Gazebo uses physics engines to simulate realistic interactions between objects. The main physics engines supported are:
- **ODE (Open Dynamics Engine)**: Default engine, good for most applications
- **Bullet**: More robust for complex contact scenarios
- **DART**: Advanced dynamics and robotics toolkit

### Physics Simulation Concepts

Gazebo simulates physics through:
- **Collision Detection**: Determining when objects intersect
- **Contact Processing**: Calculating forces when objects touch
- **Dynamics**: Computing motion based on forces and torques
- **Constraints**: Limiting motion through joints

## Setting Up a Basic Gazebo World

### Creating a World File

Gazebo world files are XML files that define the simulation environment:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sky -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics parameters -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Your robot will be spawned here -->
  </world>
</sdf>
```

### Physics Engine Parameters

Key physics parameters that affect simulation quality:

- **max_step_size**: Maximum time step for physics updates (smaller = more accurate but slower)
- **real_time_factor**: Target simulation speed relative to real time
- **real_time_update_rate**: Updates per second
- **gravity**: Gravitational acceleration vector

## Creating Physics Models

### Model with Physics Properties

Here's an example of a simple box with physics properties:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="physics_box">
    <pose>0 0 1 0 0 0</pose>

    <!-- Link definition -->
    <link name="box_link">
      <!-- Visual properties -->
      <visual name="visual">
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>

      <!-- Collision properties -->
      <collision name="collision">
        <geometry>
          <box>
            <size>0.2 0.2 0.2</size>
          </box>
        </geometry>
      </collision>

      <!-- Inertial properties -->
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.0083</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0083</iyy>
          <iyz>0</iyz>
          <izz>0.0083</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```

## Physics Properties in Detail

### Mass and Inertia

For a box with mass m and dimensions (x, y, z), the inertia values are:
- Ixx = m * (y² + z²) / 12
- Iyy = m * (x² + z²) / 12
- Izz = m * (x² + y²) / 12

```xml
<inertial>
  <mass>2.0</mass>
  <inertia>
    <ixx>0.0667</ixx>  <!-- 2.0 * (0.2² + 0.2²) / 12 -->
    <iyy>0.0667</iyy>  <!-- 2.0 * (0.2² + 0.2²) / 12 -->
    <izz>0.0667</izz>  <!-- 2.0 * (0.2² + 0.2²) / 12 -->
  </inertia>
</inertial>
```

### Friction Properties

Friction affects how objects slide against each other:

```xml
<collision name="collision">
  <geometry>
    <box><size>0.2 0.2 0.2</size></box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>      <!-- Static friction coefficient -->
        <mu2>1.0</mu2>    <!-- Secondary friction coefficient -->
        <fdir1>0 0 1</fdir1>  <!-- Friction direction -->
      </ode>
    </friction>
  </surface>
</collision>
```

### Damping and Compliance

Damping reduces oscillations and makes simulation more stable:

```xml
<collision name="collision">
  <geometry>
    <box><size>0.2 0.2 0.2</size></box>
  </geometry>
  <surface>
    <bounce>
      <restitution_coefficient>0.2</restitution_coefficient>
      <threshold>100000</threshold>
    </bounce>
    <contact>
      <ode>
        <soft_cfm>0.001</soft_cfm>
        <soft_erp>0.2</soft_erp>
        <kp>10000000</kp>
        <kd>1</kd>
      </ode>
    </contact>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

## Humanoid Robot Physics Configuration

For humanoid robots, special attention is needed for:

### Joint Dynamics

```xml
<joint name="knee_joint" type="revolute">
  <parent>thigh_link</parent>
  <child>shin_link</child>
  <axis>
    <xyz>0 1 0</xyz>
    <limit>
      <lower>0</lower>
      <upper>1.57</upper>
      <effort>100</effort>
      <velocity>1</velocity>
    </limit>
    <!-- Joint dynamics -->
    <dynamics>
      <damping>0.1</damping>
      <friction>0.05</friction>
    </dynamics>
  </axis>
</joint>
```

### Balance and Stability

For humanoid robots to maintain balance:

```xml
<!-- Torso with appropriate mass distribution -->
<link name="torso">
  <inertial>
    <mass>10.0</mass>
    <origin xyz="0 0 0.2"/>
    <inertia>
      <ixx>0.5</ixx>
      <iyy>0.3</iyy>
      <izz>0.4</izz>
    </inertia>
  </inertial>
</link>
```

## Hands-on Lab: Physics Playground

In this lab, you'll create a physics simulation with different objects to understand how physics properties affect behavior.

### Step 1: Create a Physics World

Create `physics_playground.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physics_playground">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.000001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Box with high friction -->
    <model name="high_friction_box">
      <pose>0 1 0.2 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.0083</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0083</iyy>
            <iyz>0</iyz>
            <izz>0.0083</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box><size>0.2 0.2 0.2</size></box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>0.2 0.2 0.2</size></box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>10.0</mu>
                <mu2>10.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>

    <!-- Sphere with low friction -->
    <model name="low_friction_sphere">
      <pose>0 -1 0.2 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.004</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.004</iyy>
            <iyz>0</iyz>
            <izz>0.004</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.1</mu>
                <mu2>0.1</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>

    <!-- Inclined plane -->
    <model name="inclined_plane">
      <pose>2 0 1 0 0.3 0</pose>
      <link name="link">
        <inertial>
          <mass>100.0</mass>
          <inertia>
            <ixx>100</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100</iyy>
            <iyz>0</iyz>
            <izz>100</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box><size>2 0.1 1</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box><size>2 0.1 1</size></box>
          </geometry>
        </collision>
      </link>
    </model>

  </world>
</sdf>
```

### Step 2: Launch the Simulation

```bash
# Launch Gazebo with your world
gz sim physics_playground.world
```

## Debugging Physics Simulation

### Common Physics Issues

1. **Objects falling through surfaces**: Check collision properties and world boundaries
2. **Unstable simulation**: Reduce max_step_size or adjust solver parameters
3. **Robot falling over**: Check mass distribution and joint limits
4. **Jittery movement**: Adjust damping and friction parameters

### Physics Debugging Tools

```bash
# Launch Gazebo with verbose output
gz sim -v 4 physics_playground.world

# Use Gazebo GUI to inspect physics properties
# View -> Transparent
# View -> Wireframe
# View -> Contacts
```

## Advanced Physics Features

### Buoyancy Simulation

For underwater robots:

```xml
<plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
  <link name="robot_base">0.1 0.1 0.2</link>
</plugin>
```

### Wind Simulation

For aerial or outdoor robots:

```xml>
<world name="windy_world">
  <physics type="ode">
    <!-- Physics settings -->
  </physics>

  <model name="wind_generator">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box><size>0.1 0.1 0.1</size></box>
        </geometry>
      </visual>
    </link>
  </model>

  <wind>
    <linear_velocity>2 0 0</linear_velocity>
    <factor>1.0</factor>
  </wind>
</world>
```

## Best Practices

1. **Start Simple**: Begin with basic shapes and gradually add complexity
2. **Tune Parameters**: Adjust physics parameters based on simulation behavior
3. **Validate Mass**: Ensure mass properties match real-world values
4. **Check Inertia**: Use proper inertia calculations for stable simulation
5. **Test Stability**: Run simulations for extended periods to check for drift
6. **Balance Performance**: Find the right balance between accuracy and speed

## Next Steps

After completing this chapter, you'll be ready to learn about creating robot models in Gazebo with proper simulation properties in Chapter 3.