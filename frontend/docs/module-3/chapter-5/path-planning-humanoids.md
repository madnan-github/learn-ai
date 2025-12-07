---
sidebar_position: 5
title: 'Path Planning for Humanoids'
---

# Path Planning for Humanoids

This chapter focuses on path planning specifically designed for bipedal humanoid robots, addressing the unique challenges of walking locomotion, balance, and stability that differentiate humanoid navigation from wheeled robots.

## What You'll Learn

In this chapter, you'll explore:
- Bipedal locomotion constraints in path planning
- Footstep planning algorithms
- Balance-aware path planning
- Walking pattern generation
- Terrain adaptability for humanoids
- Stability optimization techniques

## Prerequisites

- Completion of Module 1-3, Chapters 1-4
- Understanding of basic path planning algorithms
- Knowledge of humanoid robot kinematics
- Experience with ROS 2 navigation systems

## Understanding Humanoid Locomotion Constraints

### Differences from Wheeled Robots

Humanoid robots face unique challenges in navigation:

1. **Bipedal Gait**: Requires alternating foot placement
2. **Balance Requirements**: Must maintain center of mass within support polygon
3. **Limited Turning**: Cannot rotate in place like wheeled robots
4. **Step Height Limits**: Cannot step over large obstacles
5. **Terrain Sensitivity**: Requires stable footholds

### Key Constraints

- **Step Length**: Maximum distance between consecutive footsteps
- **Step Width**: Lateral distance between left and right footsteps
- **Step Height**: Maximum obstacle height that can be stepped over
- **Turning Radius**: Minimum radius for turning maneuvers
- **Stability Margin**: Required safety margin from support polygon edges

## Footstep Planning Algorithms

### Basic Footstep Planner

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
import numpy as np
import math
from collections import deque

class FootstepPlanner(Node):
    def __init__(self):
        super().__init__('footstep_planner')

        # Subscribe to navigation goal
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Publishers
        self.footstep_pub = self.create_publisher(Path, '/footsteps', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/footstep_markers', 10)

        # Robot-specific parameters
        self.step_length_max = 0.3  # meters
        self.step_width_max = 0.2  # meters
        self.turn_radius_min = 0.5  # meters
        self.step_height_max = 0.1  # meters

        # Current robot state
        self.current_pose = None
        self.current_goal = None
        self.footstep_sequence = []

        self.get_logger().info('Footstep Planner initialized')

    def goal_callback(self, msg):
        """Process navigation goal and plan footsteps"""
        self.current_goal = msg
        self.plan_footsteps()

    def plan_footsteps(self):
        """Plan sequence of footsteps from current position to goal"""
        if self.current_pose is None or self.current_goal is None:
            return

        # Calculate distance and direction to goal
        dx = self.current_goal.pose.position.x - self.current_pose.position.x
        dy = self.current_goal.pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)

        # Generate footsteps
        footsteps = self.generate_footsteps(
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.orientation,
            self.current_goal.pose.position.x,
            self.current_goal.pose.position.y,
            angle
        )

        if footsteps:
            # Create Path message
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "map"

            for step in footsteps:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = step[0]
                pose_stamped.pose.position.y = step[1]
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                path_msg.poses.append(pose_stamped)

            # Publish footsteps
            self.footstep_pub.publish(path_msg)
            self.publish_footstep_markers(path_msg)

            self.get_logger().info(f'Planned {len(footsteps)} footsteps to goal')

    def generate_footsteps(self, start_x, start_y, start_orientation, goal_x, goal_y, goal_angle):
        """Generate sequence of footsteps using simple algorithm"""
        footsteps = []

        # Current position and orientation
        current_x, current_y = start_x, start_y
        current_angle = math.atan2(goal_y - current_y, goal_x - current_x)

        # Calculate number of steps needed
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        num_steps = int(distance / self.step_length_max) + 1

        # Generate footsteps
        for i in range(num_steps):
            # Calculate next step position
            step_ratio = (i + 1) / num_steps
            next_x = start_x + dx * step_ratio
            next_y = start_y + dy * step_ratio

            # Alternate between left and right foot
            if i % 2 == 0:  # Left foot
                next_x += self.step_width_max / 2 * math.sin(current_angle)
                next_y -= self.step_width_max / 2 * math.cos(current_angle)
            else:  # Right foot
                next_x -= self.step_width_max / 2 * math.sin(current_angle)
                next_y += self.step_width_max / 2 * math.cos(current_angle)

            footsteps.append((next_x, next_y))

        return footsteps

    def publish_footstep_markers(self, path_msg):
        """Publish visualization markers for footsteps"""
        marker_array = MarkerArray()

        # Create markers for each footstep
        for i, pose_stamped in enumerate(path_msg.poses):
            # Footstep marker
            footstep_marker = Marker()
            footstep_marker.header = path_msg.header
            footstep_marker.ns = "footsteps"
            footstep_marker.id = i
            footstep_marker.type = Marker.CYLINDER
            footstep_marker.action = Marker.ADD

            footstep_marker.pose = pose_stamped.pose
            footstep_marker.scale.x = 0.1  # Diameter
            footstep_marker.scale.y = 0.1
            footstep_marker.scale.z = 0.05  # Height

            # Color based on foot (left=blue, right=red)
            if i % 2 == 0:  # Left foot
                footstep_marker.color.r = 0.0
                footstep_marker.color.g = 0.0
                footstep_marker.color.b = 1.0
            else:  # Right foot
                footstep_marker.color.r = 1.0
                footstep_marker.color.g = 0.0
                footstep_marker.color.b = 0.0
            footstep_marker.color.a = 0.7

            marker_array.markers.append(footstep_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = FootstepPlanner()

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

### Advanced Footstep Planning with Stability

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray
import numpy as np
import math
from scipy.spatial import KDTree

class StableFootstepPlanner(Node):
    def __init__(self):
        super().__init__('stable_footstep_planner')

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

        # Publishers
        self.footstep_pub = self.create_publisher(Path, '/stable_footsteps', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/stable_footstep_markers', 10)

        # Costmap data
        self.costmap = None
        self.map_resolution = 0.05
        self.map_origin = [0, 0]

        # Robot parameters
        self.step_length_max = 0.3  # meters
        self.step_width_max = 0.2
        self.foot_size = 0.15  # meters (foot length/width)
        self.support_polygon_radius = 0.1  # meters (for stability margin)

        # Current state
        self.current_pose = None
        self.current_goal = None

        self.get_logger().info('Stable Footstep Planner initialized')

    def costmap_callback(self, msg):
        """Process incoming costmap"""
        self.costmap = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_resolution = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]

    def goal_callback(self, msg):
        """Process navigation goal"""
        self.current_goal = msg
        self.plan_stable_footsteps()

    def plan_stable_footsteps(self):
        """Plan footsteps considering stability and terrain"""
        if self.costmap is None or self.current_goal is None:
            return

        # Convert goal to map coordinates
        goal_map_x = int((self.current_goal.pose.position.x - self.map_origin[0]) / self.map_resolution)
        goal_map_y = int((self.current_goal.pose.position.y - self.map_origin[1]) / self.map_resolution)

        # Convert current position to map coordinates
        if self.current_pose:
            current_map_x = int((self.current_pose.position.x - self.map_origin[0]) / self.map_resolution)
            current_map_y = int((self.current_pose.position.y - self.map_origin[1]) / self.map_resolution)
        else:
            current_map_x, current_map_y = self.costmap.shape[1] // 2, self.costmap.shape[0] // 2

        # Plan footsteps using A* with stability constraints
        footsteps = self.plan_stable_path(current_map_x, current_map_y, goal_map_x, goal_map_y)

        if footsteps:
            # Convert map coordinates back to world coordinates
            world_footsteps = []
            for x, y in footsteps:
                world_x = x * self.map_resolution + self.map_origin[0]
                world_y = y * self.map_resolution + self.map_origin[1]
                world_footsteps.append((world_x, world_y))

            # Create and publish path
            path_msg = self.create_path_message(world_footsteps)
            self.footstep_pub.publish(path_msg)
            self.publish_footstep_markers(path_msg)

            self.get_logger().info(f'Planned {len(footsteps)} stable footsteps')

    def plan_stable_path(self, start_x, start_y, goal_x, goal_y):
        """Plan path with stability constraints"""
        # Use A* algorithm but with stability-aware cost function
        from queue import PriorityQueue

        open_set = PriorityQueue()
        open_set.put((0, (start_x, start_y)))

        came_from = {}
        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): self.stability_heuristic(start_x, start_y, goal_x, goal_y)}

        while not open_set.empty():
            current = open_set.get()[1]

            if current == (goal_x, goal_y):
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            # Get valid neighbors considering step constraints
            for neighbor in self.get_stable_neighbors(current[0], current[1]):
                if neighbor[0] < 0 or neighbor[0] >= self.costmap.shape[1] or \
                   neighbor[1] < 0 or neighbor[1] >= self.costmap.shape[0]:
                    continue

                # Check if neighbor is traversable
                if self.costmap[neighbor[1], neighbor[0]] > 50:  # Obstacle
                    continue

                # Check stability of step
                if not self.is_stable_step(current[0], current[1], neighbor[0], neighbor[1]):
                    continue

                tentative_g_score = g_score[current] + self.stability_cost(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.stability_heuristic(neighbor[0], neighbor[1], goal_x, goal_y)
                    open_set.put((f_score[neighbor], neighbor))

        return None  # No path found

    def get_stable_neighbors(self, x, y):
        """Get neighbors within step constraints"""
        neighbors = []

        # Generate potential step positions within constraints
        for dx in range(-int(self.step_length_max / self.map_resolution),
                        int(self.step_length_max / self.map_resolution) + 1):
            for dy in range(-int(self.step_width_max / self.map_resolution),
                            int(self.step_width_max / self.map_resolution) + 1):
                if dx == 0 and dy == 0:
                    continue

                # Check step length constraint
                step_distance = math.sqrt(dx**2 + dy**2) * self.map_resolution
                if step_distance <= self.step_length_max:
                    neighbors.append((x + dx, y + dy))

        return neighbors

    def is_stable_step(self, from_x, from_y, to_x, to_y):
        """Check if a step maintains stability"""
        # Check if the step is within robot's capabilities
        step_dx = (to_x - from_x) * self.map_resolution
        step_dy = (to_y - from_y) * self.map_resolution
        step_distance = math.sqrt(step_dx**2 + step_dy**2)

        if step_distance > self.step_length_max:
            return False

        # Check if the target position is stable (not on an obstacle)
        if self.costmap[to_y, to_x] > 50:
            return False

        # Additional stability checks could go here
        # For example, checking terrain roughness, slope, etc.

        return True

    def stability_cost(self, pos1, pos2):
        """Calculate cost considering stability factors"""
        # Base cost is distance
        dx = (pos2[0] - pos1[0]) * self.map_resolution
        dy = (pos2[1] - pos1[1]) * self.map_resolution
        base_cost = math.sqrt(dx**2 + dy**2)

        # Add penalty for unstable terrain
        terrain_cost = self.costmap[pos2[1], pos2[0]] / 100.0  # Normalize costmap value

        # Add penalty for sharp turns (affects balance)
        if len(self.get_stable_neighbors(pos1[0], pos1[1])) > 4:  # If there are many options, prefer straight paths
            direction_cost = abs(math.atan2(dy, dx)) * 0.1
        else:
            direction_cost = 0

        return base_cost + terrain_cost + direction_cost

    def stability_heuristic(self, x1, y1, x2, y2):
        """Heuristic function considering stability"""
        dx = (x2 - x1) * self.map_resolution
        dy = (y2 - y1) * self.map_resolution
        distance = math.sqrt(dx**2 + dy**2)

        # Simple Euclidean distance heuristic
        return distance

    def create_path_message(self, footsteps):
        """Create Path message from footsteps"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for x, y in footsteps:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)

        return path_msg

    def publish_footstep_markers(self, path_msg):
        """Publish visualization markers for footsteps"""
        marker_array = MarkerArray()

        for i, pose_stamped in enumerate(path_msg.poses):
            # Footstep marker
            footstep_marker = self.create_footstep_marker(pose_stamped, i)
            marker_array.markers.append(footstep_marker)

            # Support polygon marker
            support_marker = self.create_support_polygon_marker(pose_stamped, i + len(path_msg.poses))
            marker_array.markers.append(support_marker)

        self.marker_pub.publish(marker_array)

    def create_footstep_marker(self, pose_stamped, id_num):
        """Create a marker for a single footstep"""
        marker = Marker()
        marker.header = pose_stamped.header
        marker.ns = "footsteps"
        marker.id = id_num
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose = pose_stamped.pose
        marker.scale.x = self.foot_size
        marker.scale.y = self.foot_size
        marker.scale.z = 0.02

        # Color based on foot (left=blue, right=red)
        if id_num % 2 == 0:  # Left foot
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        else:  # Right foot
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        marker.color.a = 0.8

        return marker

    def create_support_polygon_marker(self, pose_stamped, id_num):
        """Create a marker for the support polygon"""
        marker = Marker()
        marker.header = pose_stamped.header
        marker.ns = "support_polygons"
        marker.id = id_num
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose = pose_stamped.pose
        marker.scale.x = self.support_polygon_radius * 2
        marker.scale.y = self.support_polygon_radius * 2
        marker.scale.z = 0.01

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3

        return marker

def main(args=None):
    rclpy.init(args=args)
    node = StableFootstepPlanner()

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

## Walking Pattern Generation

### ZMP-Based Walking Pattern

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Pose
import numpy as np
import math

class ZMPWalkingPatternGenerator(Node):
    def __init__(self):
        super().__init__('zmp_walking_pattern_generator')

        # Publisher for walking commands
        self.walking_cmd_pub = self.create_publisher(Float32MultiArray, '/walking_commands', 10)

        # Robot parameters for ZMP (Zero Moment Point) calculation
        self.robot_height = 0.8  # meters (height of CoM)
        self.gravity = 9.81  # m/s^2
        self.step_time = 1.0  # seconds per step
        self.com_height = 0.8  # center of mass height

        # Walking parameters
        self.step_length = 0.3  # meters
        self.step_width = 0.2  # meters
        self.turn_angle = 0.0  # radians

        # Current walking state
        self.is_walking = False
        self.current_step = 0
        self.total_steps = 0

        self.get_logger().info('ZMP Walking Pattern Generator initialized')

    def generate_walking_pattern(self, step_length, step_width, num_steps, turn_angle=0.0):
        """Generate ZMP-based walking pattern"""
        # Calculate ZMP trajectory for stable walking
        omega = math.sqrt(self.gravity / self.com_height)  # Natural frequency

        # Time parameters
        dt = 0.01  # 100 Hz control rate
        step_duration = self.step_time
        double_support_duration = 0.2  # 20% of step time in double support
        single_support_duration = step_duration - double_support_duration

        # Generate complete walking pattern
        walking_pattern = []

        for step_idx in range(num_steps):
            # Determine foot positions for this step
            if step_idx % 2 == 0:  # Left foot step
                foot_x = (step_idx // 2) * step_length
                foot_y = step_width / 2
            else:  # Right foot step
                foot_x = ((step_idx - 1) // 2) * step_length
                foot_y = -step_width / 2

            # Generate ZMP trajectory for this step
            step_pattern = self.generate_step_zmp_trajectory(
                foot_x, foot_y, omega, dt, single_support_duration, double_support_duration
            )

            walking_pattern.extend(step_pattern)

        # Publish walking pattern
        self.publish_walking_commands(walking_pattern)

    def generate_step_zmp_trajectory(self, foot_x, foot_y, omega, dt, single_supp_dur, double_supp_dur):
        """Generate ZMP trajectory for a single step"""
        trajectory = []

        # Single support phase (swing foot)
        single_support_steps = int(single_supp_dur / dt)
        for i in range(single_support_steps):
            t = i * dt
            # ZMP trajectory during single support (approaches foot position)
            zmp_x = foot_x * (1 - math.exp(-omega * t)) / (1 - math.exp(-omega * single_supp_dur))
            zmp_y = foot_y
            trajectory.append([zmp_x, zmp_y, t])

        # Double support phase (both feet on ground)
        double_support_steps = int(double_supp_dur / dt)
        for i in range(double_support_steps):
            t = single_supp_dur + i * dt
            # ZMP stays at foot position during double support
            zmp_x = foot_x
            zmp_y = foot_y
            trajectory.append([zmp_x, zmp_y, t])

        return trajectory

    def publish_walking_commands(self, walking_pattern):
        """Publish walking commands to robot controller"""
        for zmp_data in walking_pattern:
            cmd_msg = Float32MultiArray()
            cmd_msg.data = zmp_data  # [zmp_x, zmp_y, time]
            self.walking_cmd_pub.publish(cmd_msg)

    def start_walking(self, linear_vel, angular_vel):
        """Start walking with specified velocities"""
        if linear_vel == 0 and angular_vel == 0:
            self.stop_walking()
            return

        # Calculate number of steps based on desired velocity
        step_frequency = 1.0 / self.step_time  # steps per second
        desired_speed = abs(linear_vel)

        if desired_speed > 0:
            self.step_length = desired_speed * self.step_time
            num_steps = int(10)  # Plan 10 steps ahead (example)
        else:
            num_steps = 0

        self.turn_angle = angular_vel * self.step_time  # Turn per step

        if num_steps > 0:
            self.generate_walking_pattern(
                self.step_length,
                self.step_width,
                num_steps,
                self.turn_angle
            )
            self.is_walking = True
            self.get_logger().info(f'Starting walk: {num_steps} steps, speed {desired_speed:.2f} m/s')

    def stop_walking(self):
        """Stop walking motion"""
        self.is_walking = False
        # Send stop command to robot
        stop_msg = Float32MultiArray()
        stop_msg.data = [0.0, 0.0, 0.0]  # [zmp_x, zmp_y, time] - stop position
        self.walking_cmd_pub.publish(stop_msg)
        self.get_logger().info('Walking stopped')

def main(args=None):
    rclpy.init(args=args)
    node = ZMPWalkingPatternGenerator()

    try:
        # Example: Start walking forward
        node.start_walking(linear_vel=0.3, angular_vel=0.0)

        # Run for a while then stop
        import time
        time.sleep(5)
        node.stop_walking()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Terrain Adaptability for Humanoids

### Terrain Classification and Adaptation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import numpy as np
import cv2
from sklearn.cluster import KMeans
from scipy.spatial import distance
import math

class TerrainAdaptationNode(Node):
    def __init__(self):
        super().__init__('terrain_adaptation_node')

        # Subscribe to sensor data
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/points2',
            self.pointcloud_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        # Publishers
        self.terrain_class_pub = self.create_publisher(String, '/terrain_classification', 10)
        self.footstep_params_pub = self.create_publisher(String, '/footstep_parameters', 10)

        # Terrain classification parameters
        self.terrain_types = {
            'flat': {'roughness_threshold': 0.01, 'slope_threshold': 0.1, 'step_height': 0.1},
            'uneven': {'roughness_threshold': 0.05, 'slope_threshold': 0.3, 'step_height': 0.05},
            'stairs': {'roughness_threshold': 0.02, 'slope_threshold': 0.5, 'step_height': 0.15},
            'obstacle': {'roughness_threshold': 0.1, 'slope_threshold': 0.2, 'step_height': 0.2}
        }

        # Current terrain classification
        self.current_terrain = 'flat'
        self.terrain_confidence = 1.0

        self.get_logger().info('Terrain Adaptation Node initialized')

    def pointcloud_callback(self, msg):
        """Process point cloud data for terrain analysis"""
        try:
            # Convert PointCloud2 to numpy array (simplified - in real system, use pcl library)
            # For this example, we'll simulate the conversion
            points_3d = self.convert_pointcloud_to_array(msg)

            if points_3d is not None and len(points_3d) > 10:
                # Analyze terrain properties
                roughness = self.calculate_surface_roughness(points_3d)
                slope = self.calculate_surface_slope(points_3d)

                # Classify terrain
                terrain_type, confidence = self.classify_terrain(roughness, slope)

                if terrain_type != self.current_terrain:
                    self.current_terrain = terrain_type
                    self.terrain_confidence = confidence

                    # Publish terrain classification
                    terrain_msg = String()
                    terrain_msg.data = f"{terrain_type}:{confidence:.2f}"
                    self.terrain_class_pub.publish(terrain_msg)

                    # Publish adapted footstep parameters
                    self.publish_adapted_parameters(terrain_type)

                    self.get_logger().info(f'Terrain classified as {terrain_type} (confidence: {confidence:.2f})')

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

    def camera_callback(self, msg):
        """Process camera data for terrain texture analysis"""
        try:
            # Convert ROS Image to OpenCV
            from cv_bridge import CvBridge
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

            # Analyze texture and color for terrain classification
            texture_features = self.extract_texture_features(cv_image)

            # This could be used to complement point cloud analysis
            # For now, we'll just log the analysis
            self.get_logger().info(f'Image texture analysis completed')

        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

    def convert_pointcloud_to_array(self, pointcloud_msg):
        """Convert PointCloud2 message to numpy array"""
        # In a real implementation, use libraries like sensor_msgs_py
        # For simulation, return a dummy array
        # This is a simplified version - real implementation would extract x, y, z coordinates

        # Simulate point cloud data
        num_points = 100
        points = np.random.rand(num_points, 3) * 2  # Random points in 2x2x2 cube
        points[:, 2] += 0.5  # Shift to positive Z

        return points

    def calculate_surface_roughness(self, points):
        """Calculate surface roughness from point cloud"""
        if len(points) < 3:
            return 0.0

        # Calculate local roughness by fitting planes to small neighborhoods
        roughness_samples = []

        # Sample neighborhoods
        sample_size = min(10, len(points))
        for i in range(0, len(points), max(1, len(points) // sample_size)):
            neighborhood = points[i:i+10] if i+10 < len(points) else points[i:]

            if len(neighborhood) >= 3:
                # Fit a plane to the neighborhood
                plane_params = self.fit_plane(neighborhood)

                # Calculate deviations from the plane
                distances = [self.point_to_plane_distance(point, plane_params)
                           for point in neighborhood]

                if distances:
                    roughness_samples.append(np.std(distances))

        return np.mean(roughness_samples) if roughness_samples else 0.0

    def fit_plane(self, points):
        """Fit a plane to a set of 3D points using SVD"""
        if len(points) < 3:
            return [0, 0, 1, 0]  # Default: horizontal plane

        # Center the points
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid

        # Calculate covariance matrix
        cov_matrix = np.cov(centered_points.T)

        # Find the normal vector (eigenvector corresponding to smallest eigenvalue)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        normal = eigenvectors[:, 0]  # Smallest eigenvalue's eigenvector

        # Calculate d parameter: ax + by + cz + d = 0
        d = -np.dot(normal, centroid)

        return [normal[0], normal[1], normal[2], d]

    def point_to_plane_distance(self, point, plane_params):
        """Calculate distance from point to plane"""
        a, b, c, d = plane_params
        x, y, z = point
        return abs(a*x + b*y + c*z + d) / math.sqrt(a*a + b*b + c*c)

    def calculate_surface_slope(self, points):
        """Calculate average surface slope"""
        if len(points) < 3:
            return 0.0

        # Fit a plane to all points
        plane_params = self.fit_plane(points)

        # The slope is related to the angle of the normal vector with the z-axis
        a, b, c, d = plane_params
        normal = np.array([a, b, c])

        # Calculate angle with z-axis (0, 0, 1)
        z_axis = np.array([0, 0, 1])
        cos_angle = np.dot(normal, z_axis) / (np.linalg.norm(normal) * np.linalg.norm(z_axis))

        # Convert to slope angle
        slope_angle = np.arccos(np.clip(cos_angle, -1, 1))

        return slope_angle

    def classify_terrain(self, roughness, slope):
        """Classify terrain based on roughness and slope"""
        best_match = 'flat'
        best_confidence = 0.0

        for terrain_type, params in self.terrain_types.items():
            # Calculate similarity score (simplified)
            roughness_score = 1.0 - min(abs(roughness - params['roughness_threshold']), 1.0)
            slope_score = 1.0 - min(abs(slope - params['slope_threshold']), 1.0)

            # Combine scores
            combined_score = (roughness_score + slope_score) / 2.0

            if combined_score > best_confidence:
                best_confidence = combined_score
                best_match = terrain_type

        return best_match, best_confidence

    def extract_texture_features(self, image):
        """Extract texture features from camera image"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Calculate local binary pattern (simplified)
        # In practice, you might use more sophisticated features
        features = {
            'mean_intensity': np.mean(gray),
            'std_intensity': np.std(gray),
            'edges': cv2.Laplacian(gray, cv2.CV_64F).var()
        }

        return features

    def publish_adapted_parameters(self, terrain_type):
        """Publish adapted footstep parameters based on terrain"""
        params = self.terrain_types[terrain_type]

        # Create parameter string
        param_str = f"step_height:{params['step_height']},slope_thresh:{params['slope_threshold']},roughness_thresh:{params['roughness_threshold']}"

        param_msg = String()
        param_msg.data = param_str
        self.footstep_params_pub.publish(param_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TerrainAdaptationNode()

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

## Balance-Aware Path Planning

### Center of Mass Optimization

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import numpy as np
import math

class BalanceAwarePlanner(Node):
    def __init__(self):
        super().__init__('balance_aware_planner')

        # Subscribe to robot state and path
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

        self.com_sub = self.create_subscription(
            PoseStamped,
            '/center_of_mass',
            self.com_callback,
            10
        )

        # Publishers
        self.adjusted_path_pub = self.create_publisher(Path, '/balance_aware_plan', 10)
        self.balance_score_pub = self.create_publisher(Float32, '/balance_score', 10)

        # Robot parameters
        self.robot_height = 0.8  # meters
        self.foot_separation = 0.2  # meters
        self.support_polygon_margin = 0.1  # meters

        # Current state
        self.original_path = None
        self.current_com = None
        self.balance_threshold = 0.8  # Minimum balance score

        self.get_logger().info('Balance-Aware Planner initialized')

    def com_callback(self, msg):
        """Update center of mass position"""
        self.current_com = msg.pose.position

    def path_callback(self, msg):
        """Process path and adjust for balance"""
        self.original_path = msg
        self.adjust_path_for_balance()

    def adjust_path_for_balance(self):
        """Adjust path considering balance constraints"""
        if self.original_path is None or self.current_com is None:
            return

        # Calculate balance-aware path
        adjusted_path = self.calculate_balance_aware_path()

        if adjusted_path:
            # Publish adjusted path
            self.adjusted_path_pub.publish(adjusted_path)

            # Calculate and publish balance score
            balance_score = self.calculate_balance_score(adjusted_path)
            score_msg = Float32()
            score_msg.data = balance_score
            self.balance_score_pub.publish(score_msg)

    def calculate_balance_aware_path(self):
        """Calculate path that maintains balance"""
        if not self.original_path.poses:
            return None

        adjusted_path = Path()
        adjusted_path.header = self.original_path.header

        for i, pose_stamped in enumerate(self.original_path.poses):
            # Calculate required foot positions to maintain balance
            adjusted_pose = self.adjust_pose_for_balance(pose_stamped, i)
            adjusted_path.poses.append(adjusted_pose)

        return adjusted_path

    def adjust_pose_for_balance(self, original_pose, step_index):
        """Adjust a single pose considering balance"""
        # Calculate desired center of mass position for this step
        desired_com_x = original_pose.pose.position.x
        desired_com_y = original_pose.pose.position.y

        # Get current center of mass
        if self.current_com:
            current_com_x = self.current_com.x
            current_com_y = self.current_com.y
        else:
            current_com_x, current_com_y = 0, 0

        # Calculate balance correction
        correction_x = (desired_com_x - current_com_x) * 0.1  # Small correction factor
        correction_y = (desired_com_y - current_com_y) * 0.1

        # Apply correction to foot position
        adjusted_pose = PoseStamped()
        adjusted_pose.header = original_pose.header
        adjusted_pose.pose.position.x = original_pose.pose.position.x + correction_x
        adjusted_pose.pose.position.y = original_pose.pose.position.y + correction_y
        adjusted_pose.pose.position.z = original_pose.pose.position.z
        adjusted_pose.pose.orientation = original_pose.pose.orientation

        return adjusted_pose

    def calculate_balance_score(self, path):
        """Calculate overall balance score for the path"""
        if not path.poses or not self.current_com:
            return 0.0

        total_stability = 0.0
        num_points = len(path.poses)

        for i, pose_stamped in enumerate(path.poses):
            # Calculate stability at this point
            stability = self.calculate_point_stability(
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                self.current_com.x if self.current_com else 0,
                self.current_com.y if self.current_com else 0
            )
            total_stability += stability

        return total_stability / num_points if num_points > 0 else 0.0

    def calculate_point_stability(self, foot_x, foot_y, com_x, com_y):
        """Calculate stability of a foot placement relative to CoM"""
        # Calculate distance from center of mass to foot position
        com_to_foot_dist = math.sqrt((com_x - foot_x)**2 + (com_y - foot_y)**2)

        # Calculate stability score (inverse relationship with distance)
        # Normalized to 0-1 range
        max_stable_distance = self.foot_separation / 2 + self.support_polygon_margin
        stability = max(0, 1 - (com_to_foot_dist / max_stable_distance))

        return stability

def main(args=None):
    rclpy.init(args=args)
    node = BalanceAwarePlanner()

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

## Hands-on Lab: Complete Humanoid Navigation System

In this lab, you'll integrate all the components to create a complete humanoid navigation system.

### Step 1: Create the Main Navigation Node

Create `humanoid_navigation_system.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray
import numpy as np
import math
from collections import deque
import threading

class HumanoidNavigationSystem(Node):
    def __init__(self):
        super().__init__('humanoid_navigation_system')

        # Subscribe to all sensors
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/points2',
            self.pointcloud_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/navigation_status', 10)
        self.path_pub = self.create_publisher(Path, '/humanoid_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/navigation_markers', 10)

        # Navigation state
        self.current_goal = None
        self.robot_pose = None
        self.imu_data = None
        self.navigation_active = False
        self.emergency_stop = False

        # Robot parameters
        self.robot_params = {
            'step_length_max': 0.3,      # meters
            'step_width_max': 0.2,       # meters
            'step_height_max': 0.1,      # meters
            'turn_radius_min': 0.5,      # meters
            'max_linear_speed': 0.3,     # m/s
            'max_angular_speed': 0.5,    # rad/s
            'com_height': 0.8,           # center of mass height
            'foot_size': 0.15            # foot size for support polygon
        }

        # Navigation buffers
        self.path_buffer = deque(maxlen=100)
        self.trajectory_buffer = deque(maxlen=50)

        # Threading lock
        self.nav_lock = threading.Lock()

        self.get_logger().info('Humanoid Navigation System initialized')

    def goal_callback(self, msg):
        """Handle navigation goal"""
        with self.nav_lock:
            self.current_goal = msg
            self.navigation_active = True

            # Publish navigation active status
            status_msg = Bool()
            status_msg.data = True
            self.status_pub.publish(status_msg)

            self.get_logger().info(f'Navigation goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        with self.nav_lock:
            self.robot_pose = msg.pose.pose

    def imu_callback(self, msg):
        """Process IMU data for balance"""
        with self.nav_lock:
            self.imu_data = msg

            # Check for dangerous tilt angles that might require emergency stop
            roll, pitch, yaw = self.quaternion_to_euler(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            )

            # Emergency stop if tilt is too high
            max_tilt = 0.5  # 30 degrees in radians
            if abs(roll) > max_tilt or abs(pitch) > max_tilt:
                self.emergency_stop = True
                self.get_logger().warn(f'Emergency stop triggered! Roll: {roll:.2f}, Pitch: {pitch:.2f}')

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        with self.nav_lock:
            if not self.navigation_active or self.emergency_stop:
                return

            # Process scan data for navigation
            self.process_scan_for_navigation(msg)

    def pointcloud_callback(self, msg):
        """Process point cloud for terrain analysis"""
        with self.nav_lock:
            # Process point cloud for terrain classification
            # This would integrate with the terrain adaptation system
            pass

    def process_scan_for_navigation(self, scan_msg):
        """Process scan data for navigation decisions"""
        # Convert scan to usable format
        angles = np.array([scan_msg.angle_min + i * scan_msg.angle_increment
                          for i in range(len(scan_msg.ranges))])
        ranges = np.array(scan_msg.ranges)

        # Filter valid ranges
        valid_mask = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max) & \
                    (~np.isnan(ranges)) & (~np.isinf(ranges))
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # Check for obstacles in path
        obstacle_detected = self.check_path_obstacles(valid_ranges, valid_angles)

        if obstacle_detected:
            # Generate avoidance command
            cmd_vel = self.generate_avoidance_command(valid_ranges, valid_angles)
        else:
            # Follow planned path
            cmd_vel = self.follow_planned_path()

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel)

    def check_path_obstacles(self, ranges, angles):
        """Check if obstacles are blocking the planned path"""
        # Define sectors to check (front, front-left, front-right)
        front_mask = (np.abs(angles) < 0.5)  # Front 60 degrees
        front_left_mask = (angles > 0.5) & (angles < 1.0)  # Front-left
        front_right_mask = (angles < -0.5) & (angles > -1.0)  # Front-right

        # Check minimum distances in each sector
        min_front = np.min(ranges[front_mask]) if np.any(front_mask) else float('inf')
        min_front_left = np.min(ranges[front_left_mask]) if np.any(front_left_mask) else float('inf')
        min_front_right = np.min(ranges[front_right_mask]) if np.any(front_right_mask) else float('inf')

        # Define safety distances
        safety_distance = 0.5  # meters

        return (min_front < safety_distance or
                min_front_left < safety_distance or
                min_front_right < safety_distance)

    def generate_avoidance_command(self, ranges, angles):
        """Generate command to avoid detected obstacles"""
        cmd_vel = Twist()

        # Find the direction with maximum clear space
        sector_size = 0.3  # radians per sector
        num_sectors = int(2 * math.pi / sector_size)

        sector_distances = []
        for i in range(num_sectors):
            sector_start = i * sector_size - math.pi
            sector_end = (i + 1) * sector_size - math.pi

            sector_mask = (angles >= sector_start) & (angles < sector_end)
            if np.any(sector_mask):
                sector_dist = np.mean(ranges[sector_mask])
            else:
                sector_dist = 0

            sector_distances.append(sector_dist)

        # Find the sector with maximum distance
        best_sector = np.argmax(sector_distances)
        best_angle = best_sector * sector_size - math.pi + sector_size / 2

        # Generate command based on best direction
        cmd_vel.linear.x = self.robot_params['max_linear_speed'] * 0.3  # Slow down
        cmd_vel.angular.z = best_angle * 1.0  # Turn toward clear direction

        # Limit angular velocity
        cmd_vel.angular.z = max(-self.robot_params['max_angular_speed'],
                               min(self.robot_params['max_angular_speed'], cmd_vel.angular.z))

        return cmd_vel

    def follow_planned_path(self):
        """Follow the planned navigation path"""
        cmd_vel = Twist()

        if self.current_goal is None or self.robot_pose is None:
            return cmd_vel

        # Calculate direction to goal
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y

        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance_to_goal = math.sqrt(dx**2 + dy**2)

        # Calculate desired direction
        desired_angle = math.atan2(dy, dx)

        # Simple proportional controller
        cmd_vel.linear.x = min(self.robot_params['max_linear_speed'],
                              max(0.1, distance_to_goal * 0.5))
        cmd_vel.angular.z = desired_angle * 1.5  # Proportional gain

        # Limit angular velocity
        cmd_vel.angular.z = max(-self.robot_params['max_angular_speed'],
                               min(self.robot_params['max_angular_speed'], cmd_vel.angular.z))

        # Check if goal is reached
        if distance_to_goal < 0.3:  # 30 cm tolerance
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.navigation_active = False

            # Publish completion status
            status_msg = Bool()
            status_msg.data = False
            self.status_pub.publish(status_msg)

            self.get_logger().info('Navigation goal reached!')

        return cmd_vel

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def publish_navigation_markers(self):
        """Publish visualization markers"""
        # Implementation would create markers for path, obstacles, etc.
        pass

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidNavigationSystem()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down humanoid navigation system...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Test the Navigation System

1. Run the complete navigation system:
```bash
python3 humanoid_navigation_system.py
```

2. Send navigation goals using RViz or command line:
```bash
# Send a goal using command line
ros2 action send_goal /navigate_to_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}"
```

## Optimization Techniques

### Adaptive Parameter Tuning

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np

class AdaptiveParameterTuner(Node):
    def __init__(self):
        super().__init__('adaptive_parameter_tuner')

        # Subscribe to performance metrics
        self.balance_sub = self.create_subscription(
            Float32,
            '/balance_score',
            self.balance_callback,
            10
        )

        self.stability_sub = self.create_subscription(
            Float32,
            '/stability_score',
            self.stability_callback,
            10
        )

        # Publisher for parameter updates
        self.param_pub = self.create_publisher(Twist, '/parameter_updates', 10)

        # Performance tracking
        self.balance_history = []
        self.stability_history = []
        self.max_history = 50

        # Current parameters
        self.current_speed_factor = 1.0
        self.current_turn_factor = 1.0

        self.get_logger().info('Adaptive Parameter Tuner initialized')

    def balance_callback(self, msg):
        """Process balance score and adjust parameters"""
        self.balance_history.append(msg.data)
        if len(self.balance_history) > self.max_history:
            self.balance_history.pop(0)

        # Adjust parameters based on balance
        avg_balance = np.mean(self.balance_history) if self.balance_history else 0.5

        if avg_balance < 0.7:  # Poor balance
            self.current_speed_factor = max(0.5, self.current_speed_factor * 0.95)
            self.current_turn_factor = max(0.7, self.current_turn_factor * 0.98)
        elif avg_balance > 0.9:  # Good balance
            self.current_speed_factor = min(1.2, self.current_speed_factor * 1.01)

    def stability_callback(self, msg):
        """Process stability score"""
        self.stability_history.append(msg.data)
        if len(self.stability_history) > self.max_history:
            self.stability_history.pop(0)

    def get_adaptive_parameters(self):
        """Get current adaptive parameters"""
        params = {
            'speed_factor': self.current_speed_factor,
            'turn_factor': self.current_turn_factor,
            'balance_score': np.mean(self.balance_history) if self.balance_history else 0.5,
            'stability_score': np.mean(self.stability_history) if self.stability_history else 0.5
        }
        return params

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveParameterTuner()

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

1. **Stability First**: Always prioritize robot stability over speed
2. **Gradual Changes**: Make smooth transitions between different walking patterns
3. **Sensor Fusion**: Combine multiple sensor inputs for robust terrain assessment
4. **Fallback Behaviors**: Implement safe stopping procedures for unexpected situations
5. **Parameter Adaptation**: Adjust walking parameters based on terrain and performance
6. **Testing**: Extensively test on various terrains before deployment

## Next Steps

After completing this chapter, you'll be ready to learn about perception systems for humanoid robots in Chapter 6.