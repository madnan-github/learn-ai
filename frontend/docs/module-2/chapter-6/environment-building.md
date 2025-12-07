---
sidebar_position: 6
title: 'Environment Building'
---

# Environment Building

This chapter covers creating and customizing simulation environments for humanoid robots, including indoor and outdoor scenarios, obstacle courses, and complex multi-room environments.

## What You'll Learn

In this chapter, you'll explore:
- Creating indoor environments (offices, homes, factories)
- Building outdoor environments (parks, streets, warehouses)
- Designing obstacle courses and testing scenarios
- Adding dynamic elements to environments
- Optimizing environments for performance

## Prerequisites

- Completion of Module 1 and 2, Chapters 1-5
- Gazebo Harmonic installed
- Basic understanding of 3D modeling concepts
- ROS 2 Humble installed

## Understanding Environment Design

### Environment Categories

Simulation environments can be categorized based on their purpose:

1. **Testing Environments**: Simple, controlled spaces for algorithm validation
2. **Training Environments**: Complex, varied spaces for AI training
3. **Benchmark Environments**: Standardized spaces for performance comparison
4. **Application Environments**: Realistic spaces for specific use cases

### Design Principles

- **Realism vs Performance**: Balance visual fidelity with simulation speed
- **Safety Margins**: Include buffer zones for robot navigation
- **Variety**: Create diverse scenarios for robust testing
- **Scalability**: Design modular environments that can be combined

## Creating Indoor Environments

### Basic Office Environment

Here's a complete office environment with furniture and obstacles:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="office_world">
    <!-- Include standard models -->
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
    </physics>

    <!-- Walls -->
    <model name="wall_1">
      <pose>0 5 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>100</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100</iyy>
            <iyz>0</iyz>
            <izz>100</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_2">
      <pose>5 0 1 0 0 1.5708</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>100</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100</iyy>
            <iyz>0</iyz>
            <izz>100</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_3">
      <pose>0 -5 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>100</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100</iyy>
            <iyz>0</iyz>
            <izz>100</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_4">
      <pose>-5 0 1 0 0 1.5708</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>100</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>100</iyy>
            <iyz>0</iyz>
            <izz>100</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Office furniture -->
    <model name="desk_1">
      <pose>2 2 0.4 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1.5 0.8 0.8</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1.5 0.8 0.8</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>20</mass>
          <inertia>
            <ixx>5</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>5</iyy>
            <iyz>0</iyz>
            <izz>5</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="chair_1">
      <pose>1.2 2 0.25 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.2</radius><length>0.5</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.2</radius><length>0.5</length></cylinder>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>0.2</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.2</iyy>
            <iyz>0</iyz>
            <izz>0.2</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Plants and decorations -->
    <model name="plant_1">
      <pose>-3 3 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.15</radius><length>1.0</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.15</radius><length>1.0</length></cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.6 0.2 1</ambient>
            <diffuse>0.2 0.6 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>2</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Doorway (space between walls) -->
    <model name="doorway_visual">
      <pose>0 4.9 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box><size>1 0.01 1</size></box>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### Multi-Room Environment

Here's a more complex environment with multiple rooms:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="multi_room_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Outer walls -->
    <model name="outer_wall_north">
      <pose>0 7.5 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>15 0.2 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>15 0.2 2</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="outer_wall_south">
      <pose>0 -7.5 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>15 0.2 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>15 0.2 2</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="outer_wall_east">
      <pose>7.5 0 1 0 0 1.5708</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>15 0.2 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>15 0.2 2</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="outer_wall_west">
      <pose>-7.5 0 1 0 0 1.5708</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>15 0.2 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>15 0.2 2</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Internal walls creating rooms -->
    <model name="internal_wall_1">
      <pose>0 2.5 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>8 0.2 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>8 0.2 2</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="internal_wall_2">
      <pose>-4 0 1 0 0 1.5708</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>4 0.2 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>4 0.2 2</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Doorways -->
    <model name="doorway_1">
      <pose>-4 2.5 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>1 0.01 1</size></box></geometry>
          <material><ambient>1 1 1 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="doorway_2">
      <pose>0 -2.5 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>3 0.01 1</size></box></geometry>
          <material><ambient>1 1 1 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Room furniture -->
    <!-- Living room furniture -->
    <model name="sofa">
      <pose>-3 5 0.3 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>2 0.8 0.6</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>2 0.8 0.6</size></box></geometry>
          <material><ambient>0.5 0.3 0.1 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Kitchen area -->
    <model name="kitchen_counter">
      <pose>4 -5 0.9 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>2 0.6 1.8</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>2 0.6 1.8</size></box></geometry>
          <material><ambient>0.8 0.8 0.9 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Bedroom area -->
    <model name="bed">
      <pose>-5 -3 0.3 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>2.2 1.6 0.6</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>2.2 1.6 0.6</size></box></geometry>
          <material><ambient>0.9 0.8 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## Creating Outdoor Environments

### Park Environment

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="park_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Terrain - using a large ground plane with texture -->
    <model name="park_ground">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>50 50</size></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>50 50</size></plane>
          </geometry>
          <material>
            <ambient>0.3 0.6 0.3 1</ambient>
            <diffuse>0.3 0.6 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Walking paths -->
    <model name="path_1">
      <pose>0 0 0.01 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>20 2 0.02</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>20 2 0.02</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="path_2">
      <pose>0 0 0.01 0 0 1.5708</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>20 2 0.02</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>20 2 0.02</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Trees -->
    <model name="tree_1">
      <pose>-5 5 1.5 0 0 0</pose>
      <static>true</static>
      <link name="trunk">
        <collision name="collision">
          <geometry><cylinder><radius>0.2</radius><length>3</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.2</radius><length>3</length></cylinder></geometry>
          <material><ambient>0.4 0.2 0.1 1</ambient></material>
        </visual>
      </link>
      <link name="leaves">
        <pose>0 0 2.5 0 0 0</pose>
        <collision name="collision">
          <geometry><sphere><radius>1.5</radius></sphere></geometry>
        </collision>
        <visual name="visual">
          <geometry><sphere><radius>1.5</radius></sphere></geometry>
          <material><ambient>0.2 0.6 0.2 1</ambient></material>
        </visual>
      </link>
      <joint name="trunk_to_leaves" type="fixed">
        <parent>trunk</parent>
        <child>leaves</child>
        <origin>0 0 2.5 0 0 0</origin>
      </joint>
    </model>

    <!-- Benches -->
    <model name="bench_1">
      <pose>8 0 0.3 0 0 0</pose>
      <static>true</static>
      <link name="base">
        <collision name="collision">
          <geometry><box><size>1.5 0.2 0.4</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.5 0.2 0.4</size></box></geometry>
          <material><ambient>0.6 0.4 0.2 1</ambient></material>
        </visual>
      </link>
      <link name="back">
        <pose>0 0 0.5 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>1.5 0.02 0.8</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.5 0.02 0.8</size></box></geometry>
          <material><ambient>0.6 0.4 0.2 1</ambient></material>
        </visual>
      </link>
      <joint name="base_to_back" type="fixed">
        <parent>base</parent>
        <child>back</child>
        <origin>0 0 0.5 0 0 0</origin>
      </joint>
    </model>

    <!-- Playground equipment -->
    <model name="playground_slide">
      <pose>-8 -5 1 0 0 0</pose>
      <static>true</static>
      <link name="base">
        <collision name="collision">
          <geometry><box><size>0.2 2 1.5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.2 2 1.5</size></box></geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
      </link>
      <link name="slide">
        <pose>0.3 0 1.2 0 0 0.3</pose>
        <collision name="collision">
          <geometry><box><size>1.5 0.1 0.2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.5 0.1 0.2</size></box></geometry>
          <material><ambient>0 0 1 1</ambient></material>
        </visual>
      </link>
      <joint name="base_to_slide" type="fixed">
        <parent>base</parent>
        <child>slide</child>
        <origin>0.3 0 1.2 0 0 0.3</origin>
      </joint>
    </model>

  </world>
</sdf>
```

## Creating Obstacle Courses

### Navigation Challenge Course

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="obstacle_course">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Start area -->
    <model name="start_area">
      <pose>0 0 0.01 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>2 2 0.02</size></box></geometry>
          <material><ambient>0 1 0 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Narrow passage -->
    <model name="narrow_passage_left">
      <pose>-0.5 3 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.1 4 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.1 4 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="narrow_passage_right">
      <pose>0.5 3 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.1 4 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.1 4 1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Pillars to navigate around -->
    <model name="pillar_1">
      <pose>2 6 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>0.3</radius><length>2</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.3</radius><length>2</length></cylinder></geometry>
          <material><ambient>0.8 0.6 0.2 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="pillar_2">
      <pose>-2 6 1 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>0.3</radius><length>2</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.3</radius><length>2</length></cylinder></geometry>
          <material><ambient>0.8 0.6 0.2 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Low barrier -->
    <model name="low_barrier">
      <pose>0 8 0.2 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>3 0.1 0.4</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>3 0.1 0.4</size></box></geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Maze section -->
    <model name="maze_wall_1">
      <pose>3 10 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.2 2 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.2 2 1</size></box></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="maze_wall_2">
      <pose>4 9 0.5 0 0 1.5708</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.2 2 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.2 2 1</size></box></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Finish area -->
    <model name="finish_area">
      <pose>0 12 0.01 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>2 2 0.02</size></box></geometry>
          <material><ambient>0 0 1 1</ambient></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## Dynamic Elements in Environments

### Moving Obstacles

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="dynamic_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Moving obstacle with prismatic joint -->
    <model name="moving_obstacle">
      <pose>0 0 0.5 0 0 0</pose>

      <link name="obstacle_base">
        <collision name="collision">
          <geometry><box><size>0.5 0.5 1</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.5 0.5 1</size></box></geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
        <inertial>
          <mass>5</mass>
          <inertia><ixx>0.5</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.5</iyy><iyz>0</iyz><izz>0.5</izz></inertia>
        </inertial>
      </link>

      <link name="track">
        <visual name="visual">
          <geometry><box><size>4 0.1 0.1</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia><ixx>100</ixx><ixy>0</ixy><ixz>0</ixz><iyy>100</iyy><iyz>0</iyz><izz>100</izz></inertia>
        </inertial>
      </link>

      <joint name="slider" type="prismatic">
        <parent>track</parent>
        <child>obstacle_base</child>
        <axis>
          <xyz>1 0 0</xyz>
          <limit>
            <lower>-1.5</lower>
            <upper>1.5</upper>
          </limit>
        </axis>
      </joint>

      <joint name="fixed_to_world" type="fixed">
        <parent>world</parent>
        <child>track</child>
        <pose>0 2 0.05 0 0 0</pose>
      </joint>

      <!-- Plugin to move the obstacle -->
      <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
        <joint_name>slider</joint_name>
        <topic>obstacle_position_command</topic>
        <update_rate>100</update_rate>
      </plugin>
    </model>

    <!-- Rotating obstacle -->
    <model name="rotating_obstacle">
      <pose>0 0 0.5 0 0 0</pose>

      <link name="rotor">
        <collision name="collision">
          <geometry><cylinder><radius>0.1</radius><length>0.1</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>0.1</radius><length>0.1</length></cylinder></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia>
        </inertial>
      </link>

      <link name="arm">
        <pose>0.3 0 0 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>0.6 0.05 0.05</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.6 0.05 0.05</size></box></geometry>
          <material><ambient>0 0 1 1</ambient></material>
        </visual>
        <inertial>
          <mass>0.5</mass>
          <inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia>
        </inertial>
      </link>

      <link name="weight">
        <pose>0.55 0 0 0 0 0</pose>
        <collision name="collision">
          <geometry><sphere><radius>0.05</radius></sphere></geometry>
        </collision>
        <visual name="visual">
          <geometry><sphere><radius>0.05</radius></sphere></geometry>
          <material><ambient>1 0.5 0 1</ambient></material>
        </visual>
        <inertial>
          <mass>0.2</mass>
          <inertia><ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz></inertia>
        </inertial>
      </link>

      <joint name="rotation_joint" type="revolute">
        <parent>rotor</parent>
        <child>arm</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>

      <joint name="weight_joint" type="fixed">
        <parent>arm</parent>
        <child>weight</child>
        <origin>0.25 0 0 0 0 0</origin>
      </joint>

      <joint name="base_joint" type="fixed">
        <parent>world</parent>
        <child>rotor</child>
        <pose>-3 0 0.05 0 0 0</pose>
      </joint>
    </model>

  </world>
</sdf>
```

## Hands-on Lab: Creating a Custom Environment

In this lab, you'll create a complete environment with various elements for humanoid robot navigation.

### Step 1: Plan Your Environment

Decide on the type of environment you want to create:
- Indoor office space
- Outdoor park
- Warehouse
- Home environment

### Step 2: Create the Environment File

Create `custom_environment.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="custom_humanoid_environment">
    <!-- Include standard models -->
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
    </physics>

    <!-- Your environment elements go here -->
    <!-- Example: A simple room with furniture -->
    <model name="room_walls">
      <pose>0 0 1 0 0 0</pose>
      <static>true</static>
      <link name="north_wall">
        <collision name="collision">
          <geometry><box><size>6 0.2 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>6 0.2 2</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
      <link name="south_wall">
        <pose>0 -3 1 0 0 0</pose>
        <collision name="collision">
          <geometry><box><size>6 0.2 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>6 0.2 2</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
      <link name="east_wall">
        <pose>3 0 1 0 0 1.5708</pose>
        <collision name="collision">
          <geometry><box><size>6 0.2 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>6 0.2 2</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
      <link name="west_wall">
        <pose>-3 0 1 0 0 1.5708</pose>
        <collision name="collision">
          <geometry><box><size>6 0.2 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>6 0.2 2</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Add furniture -->
    <model name="table">
      <pose>0 1 0.4 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1.2 0.6 0.8</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.2 0.6 0.8</size></box></geometry>
          <material><ambient>0.6 0.4 0.2 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Add some dynamic elements -->
    <model name="movable_box">
      <pose>-1 -1 0.2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.3 0.3 0.3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.3 0.3 0.3</size></box></geometry>
          <material><ambient>0 0.5 1 1</ambient></material>
        </visual>
        <inertial>
          <mass>2</mass>
          <inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia>
        </inertial>
      </link>
    </model>

  </world>
</sdf>
```

### Step 3: Test Your Environment

1. Launch Gazebo with your environment:
```bash
gz sim custom_environment.sdf
```

2. Spawn a robot to test navigation:
```bash
# In another terminal
ros2 run gazebo_ros spawn_entity.py -entity test_robot -file path/to/your/robot.urdf -x 0 -y 0 -z 0.5
```

3. Use RViz to visualize and test robot navigation in the environment.

## Environment Optimization

### Performance Tips

1. **Use Simple Geometries**: Box, cylinder, and sphere shapes perform better than complex meshes
2. **Limit Dynamic Objects**: Static objects require less computational resources
3. **Optimize Meshes**: Reduce polygon count for visual elements that don't need high detail
4. **Use Proper Scaling**: Avoid extremely large or small dimensions that can cause physics issues
5. **Collision vs Visual**: Use simpler collision geometry than visual geometry when possible

### Validation Checklist

- [ ] All objects have proper collision geometry
- [ ] No overlapping static objects
- [ ] Adequate space for robot navigation
- [ ] Proper lighting and materials
- [ ] Physics parameters are reasonable
- [ ] Environment is scalable for different robot sizes

## Best Practices

1. **Modularity**: Create reusable environment components
2. **Documentation**: Comment your SDF files clearly
3. **Testing**: Validate environments with different robot models
4. **Realism**: Base environments on real-world scenarios
5. **Performance**: Balance realism with simulation speed
6. **Safety**: Include buffer zones around obstacles

## Next Steps

After completing this chapter, you'll have mastered environment creation for humanoid robot simulation. In Module 3, you'll learn about NVIDIA Isaac Sim for advanced perception and training.