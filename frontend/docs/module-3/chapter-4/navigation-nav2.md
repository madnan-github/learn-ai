---
sidebar_position: 4
title: 'Navigation with Nav2'
---

# Navigation with Nav2

This chapter covers the Navigation2 (Nav2) framework, which provides advanced path planning and navigation capabilities for humanoid robots, including support for complex environments and dynamic obstacle avoidance.

## What You'll Learn

In this chapter, you'll explore:
- Nav2 architecture and components
- Costmap configuration for humanoid robots
- Path planning algorithms (A*, Dijkstra, DWA, etc.)
- Dynamic obstacle avoidance
- Behavior trees for navigation
- Integration with VSLAM and perception systems
- Navigation tuning and optimization

## Prerequisites

- Completion of Module 1-3, Chapters 1-3
- ROS 2 Humble installed
- Understanding of coordinate frames and TF
- Basic knowledge of path planning concepts
- Experience with sensor integration

## Understanding Nav2 Architecture

Nav2 is built on a component-based architecture with the following main components:

1. **Navigation System**: Coordinates the overall navigation process
2. **Path Planner**: Generates global paths from start to goal
3. **Controller**: Tracks the global path with local adjustments
4. **Recovery**: Handles navigation failures and stuck situations
5. **Costmap**: Maintains obstacle information for planning

### Nav2 Component Flow

```
[Goal Request] --> [Global Planner] --> [Path] --> [Controller] --> [Commands]
                     |                    |           |
                  [Costmap]          [Costmap]   [Local Planner]
                     |                    |           |
                [Sensor Data]      [Sensor Data] [Sensor Data]
```

## Nav2 Configuration for Humanoid Robots

### Basic Nav2 Configuration

```yaml
# nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the path to the Behavior Tree XML file
    default_nav_through_poses_bt_xml: "navigate_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_are_error_codes_active_condition_bt_node
    - nav2_is_path_valid_condition_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_external_condition
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node
    - nav2_speed_condition_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller configuration
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 50
      model_dt: 0.05
      batch_size: 1000
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.3
      vx_max: 0.5
      vx_min: -0.2
      vy_max: 0.5
      wz_max: 1.0
      goal_dist_tol: 0.25
      goal_angle_tol: 0.25
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      rot_stopped_velocity: 0.25
      simulation_mode: false
      model_plugin_name: "omni"
      critic_names: ["BaseObstacleCritic", "GoalCritic", "PathAlignCritic", "PathFollowCritic", "PathProgressCritic", "PreferForwardCritic"]

      BaseObstacleCritic:
        scale: 2.0
        sensor_topic: "/scan"
      GoalCritic:
        scale: 2.0
      PathAlignCritic:
        scale: 3.2
        offset: 2.0
      PathFollowCritic:
        scale: 3.0
      PathProgressCritic:
        scale: 4.0
      PreferForwardCritic:
        scale: 1.0

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      # Humanoid-specific footprint
      footprint: "[[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]"
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "turtlebot3_world.yaml"

map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

recoveries_server:
  ros__parameters:
    costmap_topic: "local_costmap/costmap_raw"
    footprint_topic: "local_costmap/published_footprint"
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
      sim_frequency: 5
      angle_thresh: 0.0
      time_allowance: 10
    backup:
      plugin: "nav2_recoveries::BackUp"
      sim_frequency: 5
      distance: 0.15
      speed: 0.025
      time_allowance: 10
    wait:
      plugin: "nav2_recoveries::Wait"
      sim_frequency: 5
      time: 5
```

## Path Planning Algorithms

### Nav2 Path Planner Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.spatial import KDTree
import heapq

class Nav2PathPlanner(Node):
    def __init__(self):
        super().__init__('nav2_path_planner')

        # Subscribe to costmap and goal
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Publisher for planned path
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)

        # Store costmap data
        self.costmap = None
        self.map_resolution = 0.05
        self.map_origin = [0, 0]
        self.current_goal = None

        self.get_logger().info('Nav2 Path Planner initialized')

    def costmap_callback(self, msg):
        """Process incoming costmap"""
        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]

    def goal_callback(self, msg):
        """Process incoming goal and plan path"""
        if self.costmap is not None:
            self.current_goal = msg
            self.plan_path()

    def plan_path(self):
        """Plan path using A* algorithm"""
        if self.costmap is None or self.current_goal is None:
            return

        # Convert goal position to map coordinates
        goal_x = int((self.current_goal.pose.position.x - self.map_origin[0]) / self.map_resolution)
        goal_y = int((self.current_goal.pose.position.y - self.map_origin[1]) / self.map_resolution)

        # Get robot position (simplified - in real system, use AMCL)
        robot_x, robot_y = self.get_robot_position()

        # Run A* path planning
        path = self.a_star_path(robot_x, robot_y, goal_x, goal_y)

        if path:
            # Convert path to ROS Path message
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"

            for point in path:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = point[0] * self.map_resolution + self.map_origin[0]
                pose_stamped.pose.position.y = point[1] * self.map_resolution + self.map_origin[1]
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                path_msg.poses.append(pose_stamped)

            # Publish path
            self.path_pub.publish(path_msg)

            # Publish visualization markers
            self.publish_path_markers(path_msg)

            self.get_logger().info(f'Path planned with {len(path)} waypoints')

    def a_star_path(self, start_x, start_y, goal_x, goal_y):
        """A* path planning algorithm"""
        if (start_x < 0 or start_x >= self.costmap.shape[1] or
            start_y < 0 or start_y >= self.costmap.shape[0] or
            goal_x < 0 or goal_x >= self.costmap.shape[1] or
            goal_y < 0 or goal_y >= self.costmap.shape[0]):
            return None

        # Check if start or goal is in obstacle
        if self.costmap[start_y, start_x] > 50 or self.costmap[goal_y, goal_x] > 50:
            return None

        # A* algorithm implementation
        open_set = [(0, (start_x, start_y))]
        came_from = {}
        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): self.heuristic(start_x, start_y, goal_x, goal_y)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == (goal_x, goal_y):
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current[0], current[1]):
                if neighbor[0] < 0 or neighbor[0] >= self.costmap.shape[1] or \
                   neighbor[1] < 0 or neighbor[1] >= self.costmap.shape[0]:
                    continue

                # Skip if obstacle
                if self.costmap[neighbor[1], neighbor[0]] > 50:
                    continue

                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor[0], neighbor[1], goal_x, goal_y)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found

    def heuristic(self, x1, y1, x2, y2):
        """Heuristic function for A* (Euclidean distance)"""
        return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def distance(self, pos1, pos2):
        """Distance between two positions"""
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def get_neighbors(self, x, y):
        """Get 8-connected neighbors"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbors.append((x + dx, y + dy))
        return neighbors

    def get_robot_position(self):
        """Get robot position (simplified - in real system, use TF or AMCL)"""
        # For simulation, return a fixed position
        # In real system, use TF to get robot pose in map frame
        return 10, 10  # Example coordinates

    def publish_path_markers(self, path_msg):
        """Publish visualization markers for path"""
        marker_array = MarkerArray()

        # Create line strip marker for path
        path_marker = Marker()
        path_marker.header = path_msg.header
        path_marker.ns = "navigation_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD

        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.05  # Line width

        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0

        for pose_stamped in path_msg.poses:
            point = Point()
            point.x = pose_stamped.pose.position.x
            point.y = pose_stamped.pose.position.y
            point.z = pose_stamped.pose.position.z
            path_marker.points.append(point)

        marker_array.markers.append(path_marker)

        # Create start and goal markers
        if len(path_msg.poses) >= 2:
            start_marker = Marker()
            start_marker.header = path_msg.header
            start_marker.ns = "navigation_start"
            start_marker.id = 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD

            start_marker.pose = path_msg.poses[0].pose
            start_marker.scale.x = 0.3
            start_marker.scale.y = 0.3
            start_marker.scale.z = 0.3

            start_marker.color.r = 0.0
            start_marker.color.g = 0.0
            start_marker.color.b = 1.0
            start_marker.color.a = 1.0

            marker_array.markers.append(start_marker)

            goal_marker = Marker()
            goal_marker.header = path_msg.header
            goal_marker.ns = "navigation_goal"
            goal_marker.id = 2
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD

            goal_marker.pose = path_msg.poses[-1].pose
            goal_marker.scale.x = 0.3
            goal_marker.scale.y = 0.3
            goal_marker.scale.z = 0.3

            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0

            marker_array.markers.append(goal_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2PathPlanner()

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

## Costmap Configuration for Humanoid Robots

### Advanced Costmap Configuration

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PointStamped
import numpy as np
import cv2
from scipy.ndimage import binary_dilation, binary_erosion

class HumanoidCostmapGenerator(Node):
    def __init__(self):
        super().__init__('humanoid_costmap_generator')

        # Subscribe to sensor data
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/points2',
            self.pointcloud_callback,
            10
        )

        # Publisher for costmap
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/custom_costmap', 10)

        # Costmap parameters
        self.map_width = 20  # meters
        self.map_height = 20  # meters
        self.resolution = 0.1  # meters per cell
        self.map_size_x = int(self.map_width / self.resolution)
        self.map_size_y = int(self.map_height / self.resolution)

        # Robot-specific parameters for humanoid
        self.robot_radius = 0.3  # meters
        self.robot_height = 1.5  # meters (for 3D considerations)

        # Initialize costmap
        self.costmap = np.zeros((self.map_size_y, self.map_size_x), dtype=np.int8)

        # Robot position (simplified - in real system, use TF)
        self.robot_x = self.map_size_x // 2
        self.robot_y = self.map_size_y // 2

        self.get_logger().info('Humanoid Costmap Generator initialized')

    def laser_callback(self, msg):
        """Process laser scan data and update costmap"""
        # Convert laser scan to occupancy grid
        ranges = np.array(msg.ranges)
        angles = np.array([msg.angle_min + i * msg.angle_increment for i in range(len(ranges))])

        # Filter out invalid ranges
        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max) & (~np.isnan(ranges)) & (~np.isinf(ranges))
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # Convert to Cartesian coordinates relative to robot
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)

        # Convert to map coordinates
        map_x = (x_points / self.resolution + self.robot_x).astype(int)
        map_y = (y_points / self.resolution + self.robot_y).astype(int)

        # Filter points within map bounds
        valid_points = (map_x >= 0) & (map_x < self.map_size_x) & \
                      (map_y >= 0) & (map_y < self.map_size_y)

        map_x = map_x[valid_points]
        map_y = map_y[valid_points]

        # Update costmap with obstacle information
        self.update_costmap_obstacles(map_x, map_y)

        # Publish updated costmap
        self.publish_costmap()

    def pointcloud_callback(self, msg):
        """Process point cloud data and update costmap"""
        # In a real system, this would convert PointCloud2 to numpy array
        # For this example, we'll simulate the process
        self.get_logger().info('Point cloud received (simulation)')

    def update_costmap_obstacles(self, obstacle_x, obstacle_y):
        """Update costmap with obstacle information"""
        # Clear previous obstacle information (set to -1 for unknown)
        self.costmap[:] = -1

        # Mark obstacles
        for x, y in zip(obstacle_x, obstacle_y):
            if 0 <= x < self.map_size_x and 0 <= y < self.map_size_y:
                # Mark as obstacle (100)
                self.costmap[y, x] = 100

                # Apply inflation based on humanoid robot size
                self.inflate_obstacle(x, y, int(self.robot_radius / self.resolution))

    def inflate_obstacle(self, center_x, center_y, inflation_radius):
        """Inflate obstacles to account for robot size"""
        # Create a circular mask for inflation
        y, x = np.ogrid[:self.map_size_y, :self.map_size_x]
        mask = (x - center_x)**2 + (y - center_y)**2 <= inflation_radius**2

        # Apply inflation to costmap
        self.costmap[mask & (self.costmap != 100)] = 99  # High cost but not maximum

    def publish_costmap(self):
        """Publish costmap as OccupancyGrid message"""
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = "map"

        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.map_size_x
        costmap_msg.info.height = self.map_size_y
        costmap_msg.info.origin.position.x = -self.map_width / 2.0
        costmap_msg.info.origin.position.y = -self.map_height / 2.0
        costmap_msg.info.origin.position.z = 0.0
        costmap_msg.info.origin.orientation.w = 1.0

        # Flatten costmap array and convert to list
        costmap_data = self.costmap.flatten()
        costmap_msg.data = costmap_data.tolist()

        self.costmap_pub.publish(costmap_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidCostmapGenerator()

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

## Behavior Trees for Navigation

### Nav2 Behavior Tree Configuration

```xml
<!-- navigate_w_replanning_and_recovery.xml -->
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <PipelineSequence name="NavigateWithReplanning">
            <RateController hz="1.0">
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            </RateController>
            <RecoveryNode number_of_retries="6" name="NavigateRecovery">
                <PipelineSequence name="NavigateRecoveryActions">
                    <ClearEntireCostmap name="ClearGlobalCostmap-1" service_name="global_costmap/clear_entirely_global_costmap"/>
                    <ClearEntireCostmap name="ClearLocalCostmap-1" service_name="local_costmap/clear_entirely_local_costmap"/>
                    <Spin name="Spin" spin_dist="1.57"/>
                </PipelineSequence>
            </RecoveryNode>
        </PipelineSequence>
        <ReactiveSequence name="FollowPathWithReplanning">
            <IsPathValid path="{path}"/>
            <FollowPath path="{path}" controller_id="FollowPath"/>
        </ReactiveSequence>
    </BehaviorTree>
</root>
```

### Custom Behavior Tree Node for Humanoid Navigation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
import time

class HumanoidNavigateToPoseServer(Node):
    def __init__(self):
        super().__init__('humanoid_navigate_to_pose_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

        # Publisher for navigation status
        self.nav_status_pub = self.create_publisher(Bool, '/navigation_active', 10)

        self.get_logger().info('Humanoid Navigate To Pose Server initialized')

    def execute_callback(self, goal_handle):
        """Execute navigation goal"""
        self.get_logger().info('Received navigation goal')

        # Publish navigation active status
        status_msg = Bool()
        status_msg.data = True
        self.nav_status_pub.publish(status_msg)

        # Get goal pose
        goal_pose = goal_handle.request.pose

        # Simulate navigation execution
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()

        # Navigation simulation
        current_pose = self.get_current_pose()
        distance = self.calculate_distance(current_pose, goal_pose)

        # Simulate navigation steps
        for i in range(10):  # Simulate 10 steps
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                status_msg.data = False
                self.nav_status_pub.publish(status_msg)
                return result

            # Update feedback
            feedback_msg.current_pose = current_pose
            feedback_msg.distance_remaining = distance * (10 - i) / 10.0
            goal_handle.publish_feedback(feedback_msg)

            # Simulate movement
            time.sleep(0.5)

        # Navigation completed
        goal_handle.succeed()
        result.result = True
        self.get_logger().info('Navigation completed successfully')

        # Publish navigation inactive status
        status_msg.data = False
        self.nav_status_pub.publish(status_msg)

        return result

    def get_current_pose(self):
        """Get current robot pose (simplified)"""
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        return pose

    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        dz = pose2.position.z - pose1.position.z
        return (dx**2 + dy**2 + dz**2)**0.5

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidNavigateToPoseServer()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Dynamic Obstacle Avoidance

### Dynamic Obstacle Detection and Avoidance

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
import numpy as np
from collections import deque
import math

class DynamicObstacleAvoider(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_avoider')

        # Subscribe to laser scan and path
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.avoidance_marker_pub = self.create_publisher(MarkerArray, '/avoidance_markers', 10)

        # Store path and current position
        self.current_path = None
        self.path_index = 0
        self.obstacle_buffer = deque(maxlen=10)  # Store recent obstacle data

        # Robot parameters
        self.robot_radius = 0.3  # meters
        self.safe_distance = 0.5  # meters
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s

        # Navigation state
        self.is_avoiding = False
        self.avoidance_start_time = None

        self.get_logger().info('Dynamic Obstacle Avoider initialized')

    def path_callback(self, msg):
        """Store the current navigation path"""
        self.current_path = msg
        self.path_index = 0  # Reset path index

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection and avoidance"""
        try:
            # Convert scan to Cartesian points
            angles = np.array([msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))])
            ranges = np.array(msg.ranges)

            # Filter valid ranges
            valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max) & (~np.isnan(ranges)) & (~np.isinf(ranges))
            valid_ranges = ranges[valid_mask]
            valid_angles = angles[valid_mask]

            # Convert to Cartesian coordinates
            x_points = valid_ranges * np.cos(valid_angles)
            y_points = valid_ranges * np.sin(valid_angles)

            # Detect obstacles in front of robot
            front_mask = (np.abs(y_points) < 0.5) & (x_points > 0) & (x_points < 2.0)  # Front half-plane
            front_obstacles_x = x_points[front_mask]
            front_obstacles_y = y_points[front_mask]

            if len(front_obstacles_x) > 0:
                # Check if obstacles are too close
                min_distance = np.min(np.sqrt(front_obstacles_x**2 + front_obstacles_y**2))

                if min_distance < self.safe_distance:
                    self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m, initiating avoidance')
                    cmd_vel = self.avoid_obstacle(front_obstacles_x, front_obstacles_y)
                    self.cmd_vel_pub.publish(cmd_vel)
                    self.is_avoiding = True
                    self.avoidance_start_time = self.get_clock().now().nanoseconds / 1e9
                    return
                else:
                    self.is_avoiding = False
            else:
                self.is_avoiding = False

            # If not avoiding, follow path
            if not self.is_avoiding:
                cmd_vel = self.follow_path()
                self.cmd_vel_pub.publish(cmd_vel)

        except Exception as e:
            self.get_logger().error(f'Error in scan processing: {str(e)}')

    def avoid_obstacle(self, obs_x, obs_y):
        """Generate velocity commands to avoid obstacles"""
        cmd_vel = Twist()

        # Calculate obstacle centroid
        centroid_x = np.mean(obs_x)
        centroid_y = np.mean(obs_y)

        # Determine avoidance direction (prefer left if obstacle is on the right, vice versa)
        if centroid_y > 0:  # Obstacle on the right
            cmd_vel.angular.z = self.max_angular_speed  # Turn left
            cmd_vel.linear.x = self.max_linear_speed * 0.3  # Slow down
        else:  # Obstacle on the left
            cmd_vel.angular.z = -self.max_angular_speed  # Turn right
            cmd_vel.linear.x = self.max_linear_speed * 0.3  # Slow down

        # Limit angular velocity based on proximity
        distance = np.sqrt(centroid_x**2 + centroid_y**2)
        cmd_vel.angular.z *= (distance / self.safe_distance)  # Reduce turn as we get closer

        return cmd_vel

    def follow_path(self):
        """Follow the planned path"""
        cmd_vel = Twist()

        if self.current_path is None or len(self.current_path.poses) == 0:
            return cmd_vel

        # Get current robot position (simplified - in real system, use TF)
        robot_x, robot_y = 0.0, 0.0  # Placeholder

        # Find next waypoint to follow
        target_x = self.current_path.poses[self.path_index].pose.position.x
        target_y = self.current_path.poses[self.path_index].pose.position.y

        # Calculate distance to target
        dist_to_target = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)

        # If close to current target, move to next waypoint
        if dist_to_target < 0.3 and self.path_index < len(self.current_path.poses) - 1:
            self.path_index += 1
            target_x = self.current_path.poses[self.path_index].pose.position.x
            target_y = self.current_path.poses[self.path_index].pose.position.y

        # Calculate direction to target
        angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)

        # Simple proportional controller
        cmd_vel.linear.x = min(self.max_linear_speed, dist_to_target)
        cmd_vel.angular.z = angle_to_target * 1.0  # Proportional gain

        # Limit angular velocity
        cmd_vel.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd_vel.angular.z))

        return cmd_vel

    def publish_avoidance_markers(self):
        """Publish visualization markers for avoidance behavior"""
        marker_array = MarkerArray()

        # Create markers for dynamic obstacles
        # Implementation would go here

        self.avoidance_marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleAvoider()

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

## Hands-on Lab: Complete Navigation System

In this lab, you'll create a complete navigation system that integrates path planning, obstacle avoidance, and humanoid-specific considerations.

### Step 1: Create the Navigation System Launch File

Create `nav2_humanoid_launch.py`:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file', default='nav2_params.yaml')

    # Include Nav2 main launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file
        }.items()
    )

    # Humanoid-specific navigation nodes
    humanoid_path_planner = Node(
        package='ai_robo_learning',
        executable='nav2_path_planner',
        name='humanoid_path_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/plan', '/humanoid_plan'),
            ('/goal_pose', '/goal_pose')
        ]
    )

    humanoid_costmap_generator = Node(
        package='ai_robo_learning',
        executable='humanoid_costmap_generator',
        name='humanoid_costmap_generator',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/scan', '/scan'),
            ('/custom_costmap', '/humanoid_costmap')
        ]
    )

    dynamic_obstacle_avoider = Node(
        package='ai_robo_learning',
        executable='dynamic_obstacle_avoider',
        name='dynamic_obstacle_avoider',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/scan', '/scan'),
            ('/plan', '/humanoid_plan'),
            ('/cmd_vel', '/cmd_vel')
        ]
    )

    # Return launch description
    ld = LaunchDescription()

    # Add all actions
    ld.add_action(nav2_launch)
    ld.add_action(humanoid_path_planner)
    ld.add_action(humanoid_costmap_generator)
    ld.add_action(dynamic_obstacle_avoider)

    return ld
```

### Step 2: Create the Complete Navigation Node

Create `complete_navigation_system.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import Bool
import numpy as np
from collections import deque
import math
import threading

class CompleteNavigationSystem(Node):
    def __init__(self):
        super().__init__('complete_navigation_system')

        # Subscribe to sensors
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/navigation_active', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/nav_markers', 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Navigation state
        self.current_path = None
        self.path_index = 0
        self.current_goal = None
        self.robot_pose = None
        self.imu_data = None
        self.navigation_active = False

        # Robot parameters
        self.robot_radius = 0.3  # meters
        self.safe_distance = 0.5  # meters
        self.max_linear_speed = 0.3  # m/s (slower for humanoid stability)
        self.max_angular_speed = 0.5  # rad/s
        self.min_linear_speed = 0.1  # minimum speed to maintain stability

        # Obstacle avoidance
        self.obstacle_buffer = deque(maxlen=20)
        self.avoidance_active = False
        self.last_avoidance_time = 0

        # Threading lock
        self.nav_lock = threading.Lock()

        self.get_logger().info('Complete Navigation System initialized')

    def goal_callback(self, msg):
        """Handle navigation goal"""
        with self.nav_lock:
            self.current_goal = msg
            self.navigation_active = True
            self.path_index = 0
            self.avoidance_active = False

            # Publish navigation status
            status_msg = Bool()
            status_msg.data = True
            self.status_pub.publish(status_msg)

            self.get_logger().info(f'Navigation goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def path_callback(self, msg):
        """Store navigation path"""
        with self.nav_lock:
            self.current_path = msg
            self.path_index = 0

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        with self.nav_lock:
            self.robot_pose = msg.pose.pose

    def imu_callback(self, msg):
        """Store IMU data"""
        with self.nav_lock:
            self.imu_data = msg

    def scan_callback(self, msg):
        """Process laser scan for navigation"""
        with self.nav_lock:
            if not self.navigation_active:
                return

            # Convert scan to obstacle points
            angles = np.array([msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))])
            ranges = np.array(msg.ranges)

            # Filter valid ranges
            valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max) & (~np.isnan(ranges)) & (~np.isinf(ranges))
            valid_ranges = ranges[valid_mask]
            valid_angles = angles[valid_mask]

            # Convert to Cartesian coordinates
            x_points = valid_ranges * np.cos(valid_angles)
            y_points = valid_ranges * np.sin(valid_angles)

            # Check for obstacles in robot's path
            obstacle_detected = self.check_path_obstacles(x_points, y_points)

            if obstacle_detected:
                # Generate avoidance command
                cmd_vel = self.generate_avoidance_command(x_points, y_points)
                self.avoidance_active = True
                self.last_avoidance_time = self.get_clock().now().nanoseconds / 1e9
            else:
                # Follow path normally
                cmd_vel = self.follow_path_command()
                self.avoidance_active = False

            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)

    def check_path_obstacles(self, obs_x, obs_y):
        """Check if obstacles are blocking the current path"""
        if self.current_path is None or self.robot_pose is None:
            return False

        # Get current position
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y

        # Check next few waypoints
        check_distance = 1.0  # meters ahead to check
        path_points = []

        for i in range(self.path_index, min(self.path_index + 10, len(self.current_path.poses))):
            pose = self.current_path.poses[i].pose.position
            path_points.append((pose.x, pose.y))

        # Check if any obstacles are within safe distance of path
        for obs_x_single, obs_y_single in zip(obs_x, obs_y):
            # Convert obstacle to map coordinates relative to robot
            world_obs_x = robot_x + obs_x_single
            world_obs_y = robot_y + obs_y_single

            # Check distance to path
            for path_x, path_y in path_points:
                dist = math.sqrt((world_obs_x - path_x)**2 + (world_obs_y - path_y)**2)
                if dist < self.safe_distance:
                    return True

        return False

    def generate_avoidance_command(self, obs_x, obs_y):
        """Generate command to avoid obstacles"""
        cmd_vel = Twist()

        # Calculate obstacle centroid
        if len(obs_x) > 0:
            centroid_x = np.mean(obs_x)
            centroid_y = np.mean(obs_y)

            # Determine avoidance direction
            if centroid_y > 0:  # Obstacle on the right
                cmd_vel.angular.z = self.max_angular_speed * 0.7
            else:  # Obstacle on the left
                cmd_vel.angular.z = -self.max_angular_speed * 0.7

            # Reduce linear speed during avoidance
            cmd_vel.linear.x = self.max_linear_speed * 0.3

        return cmd_vel

    def follow_path_command(self):
        """Generate command to follow the planned path"""
        cmd_vel = Twist()

        if (self.current_path is None or
            self.robot_pose is None or
            len(self.current_path.poses) == 0):
            return cmd_vel

        # Get current robot position
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y

        # Find the closest waypoint
        min_dist = float('inf')
        closest_idx = self.path_index

        for i in range(self.path_index, len(self.current_path.poses)):
            pose = self.current_path.poses[i].pose.position
            dist = math.sqrt((pose.x - robot_x)**2 + (pose.y - robot_y)**2)

            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Update path index to closest point
        self.path_index = closest_idx

        # Get target waypoint (a few steps ahead for smoother navigation)
        target_idx = min(self.path_index + 3, len(self.current_path.poses) - 1)
        target_pose = self.current_path.poses[target_idx].pose.position

        # Calculate direction to target
        dx = target_pose.x - robot_x
        dy = target_pose.y - robot_y
        target_angle = math.atan2(dy, dx)

        # Calculate distance to target
        dist_to_target = math.sqrt(dx**2 + dy**2)

        # Simple proportional controller
        cmd_vel.linear.x = max(self.min_linear_speed,
                              min(self.max_linear_speed, dist_to_target * 0.5))
        cmd_vel.angular.z = target_angle * 1.5  # Proportional gain

        # Limit angular velocity
        cmd_vel.angular.z = max(-self.max_angular_speed,
                               min(self.max_angular_speed, cmd_vel.angular.z))

        # Check if we've reached the goal
        goal_pose = self.current_path.poses[-1].pose.position
        goal_dist = math.sqrt((goal_pose.x - robot_x)**2 + (goal_pose.y - robot_y)**2)

        if goal_dist < 0.3:  # Close enough to goal
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.navigation_active = False

            # Publish completion status
            status_msg = Bool()
            status_msg.data = False
            self.status_pub.publish(status_msg)

            self.get_logger().info('Navigation goal reached!')

        return cmd_vel

    def publish_navigation_markers(self):
        """Publish visualization markers for navigation"""
        marker_array = MarkerArray()

        # Implementation would create markers for path, obstacles, etc.

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = CompleteNavigationSystem()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down navigation system...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Test the Navigation System

1. Run the complete navigation system:
```bash
python3 complete_navigation_system.py
```

2. Send navigation goals using RViz or command line:
```bash
# Send a goal using command line
ros2 action send_goal /navigate_to_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}"
```

## Tuning and Optimization

### Navigation Parameter Tuning

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from std_msgs.msg import String

class NavigationTuner(Node):
    def __init__(self):
        super().__init__('navigation_tuner')

        # Declare parameters with descriptions
        self.declare_parameter('linear_vel_max', 0.5,
                              ParameterDescriptor(description='Maximum linear velocity'))
        self.declare_parameter('angular_vel_max', 1.0,
                              ParameterDescriptor(description='Maximum angular velocity'))
        self.declare_parameter('safe_distance', 0.5,
                              ParameterDescriptor(description='Minimum safe distance to obstacles'))
        self.declare_parameter('inflation_radius', 0.55,
                              ParameterDescriptor(description='Costmap inflation radius'))

        # Publisher for tuning updates
        self.tuning_pub = self.create_publisher(String, '/tuning_updates', 10)

        # Timer for parameter monitoring
        self.timer = self.create_timer(1.0, self.check_parameters)

        self.get_logger().info('Navigation Tuner initialized')

    def check_parameters(self):
        """Monitor and report parameter values"""
        linear_vel = self.get_parameter('linear_vel_max').value
        angular_vel = self.get_parameter('angular_vel_max').value
        safe_dist = self.get_parameter('safe_distance').value
        inflation = self.get_parameter('inflation_radius').value

        # Log current values
        self.get_logger().info(
            f'Current parameters - Linear: {linear_vel}, '
            f'Angular: {angular_vel}, Safe dist: {safe_dist}, '
            f'Inflation: {inflation}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = NavigationTuner()

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

## Best Practices

1. **Safety First**: Always maintain safe distances from obstacles
2. **Stability**: Use appropriate speeds for humanoid robot stability
3. **Sensor Fusion**: Combine multiple sensor sources for robust navigation
4. **Path Smoothing**: Smooth paths to reduce jerky movements
5. **Recovery Behaviors**: Implement robust recovery behaviors for stuck situations
6. **Parameter Tuning**: Continuously tune parameters based on performance

## Next Steps

After completing this chapter, you'll be ready to learn about path planning specifically for bipedal humanoid movement in Chapter 5.