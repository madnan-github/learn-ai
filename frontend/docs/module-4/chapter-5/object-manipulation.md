---
sidebar_position: 5
title: 'Object Manipulation'
---

# Object Manipulation

This chapter covers robotic object manipulation, including grasping, picking, placing, and manipulation planning that enables humanoid robots to interact with objects in their environment.

## What You'll Learn

In this chapter, you'll explore:
- Robot kinematics and inverse kinematics for manipulation
- Grasping and grasp planning algorithms
- Manipulation planning and execution
- Force control and tactile feedback
- Object recognition for manipulation
- Multi-fingered hand control
- Manipulation safety and validation

## Prerequisites

- Completion of Module 1-4, Chapters 1-4
- Understanding of ROS 2 messaging and action servers
- Knowledge of robot kinematics
- Experience with Python and robotics libraries
- Understanding of coordinate systems and transformations

## Robot Kinematics for Manipulation

### Forward and Inverse Kinematics

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

class ManipulationKinematics(Node):
    def __init__(self):
        super().__init__('manipulation_kinematics')

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers
        self.end_effector_pub = self.create_publisher(Pose, '/end_effector_pose', 10)
        self.ik_solution_pub = self.create_publisher(Float32MultiArray, '/ik_solution', 10)

        # Robot parameters (example for 6-DOF manipulator)
        self.dh_parameters = [
            # [a, alpha, d, theta_offset]
            [0.0, math.pi/2, 0.1, 0.0],    # Joint 1
            [0.4, 0.0, 0.0, 0.0],          # Joint 2
            [0.0, math.pi/2, 0.0, 0.0],    # Joint 3
            [0.0, -math.pi/2, 0.4, 0.0],   # Joint 4
            [0.0, math.pi/2, 0.0, 0.0],    # Joint 5
            [0.0, 0.0, 0.1, 0.0]           # Joint 6
        ]

        # Current joint angles
        self.current_joints = np.zeros(6)

        # Kinematics cache
        self.fk_cache = None

        self.get_logger().info('Manipulation Kinematics initialized')

    def joint_state_callback(self, msg):
        """Update joint states"""
        if len(msg.position) >= 6:
            self.current_joints = np.array(msg.position[:6])
            # Calculate forward kinematics
            ee_pose = self.forward_kinematics(self.current_joints)
            if ee_pose is not None:
                self.end_effector_pub.publish(ee_pose)

    def dh_transform(self, a, alpha, d, theta):
        """Calculate Denavit-Hartenberg transformation matrix"""
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        st = math.sin(theta)
        ct = math.cos(theta)

        transform = np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
        return transform

    def forward_kinematics(self, joint_angles):
        """Calculate forward kinematics"""
        try:
            # Calculate transformation matrices for each joint
            T = np.eye(4)  # Identity matrix

            for i in range(len(self.dh_parameters)):
                a, alpha, d, theta_offset = self.dh_parameters[i]
                theta = joint_angles[i] + theta_offset
                Ti = self.dh_transform(a, alpha, d, theta)
                T = T @ Ti

            # Extract position and orientation
            position = T[:3, 3]
            rotation_matrix = T[:3, :3]

            # Convert rotation matrix to quaternion
            quat = self.rotation_matrix_to_quaternion(rotation_matrix)

            # Create Pose message
            pose = Pose()
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])

            return pose

        except Exception as e:
            self.get_logger().error(f'Forward kinematics error: {str(e)}')
            return None

    def inverse_kinematics(self, target_pose):
        """Calculate inverse kinematics (simplified analytical solution)"""
        try:
            # Extract target position
            target_pos = np.array([
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z
            ])

            # Simplified analytical IK for 6-DOF manipulator
            # This is a highly simplified version - real IK would be more complex
            x, y, z = target_pos

            # Calculate joint angles (simplified example)
            theta1 = math.atan2(y, x)

            # Calculate distance to target in XY plane
            r = math.sqrt(x**2 + y**2)

            # Calculate Z distance
            z_eff = z - self.dh_parameters[0][2]  # Subtract base height

            # Calculate remaining joint angles based on arm geometry
            # This is a simplified calculation
            l2 = self.dh_parameters[1][0]  # Upper arm length
            l3 = self.dh_parameters[3][2]  # Forearm length

            # Distance from shoulder to target
            d = math.sqrt(r**2 + z_eff**2)

            # Shoulder angle
            cos_theta2 = (l2**2 + d**2 - l3**2) / (2 * l2 * d)
            cos_theta2 = max(-1, min(1, cos_theta2))  # Clamp to [-1, 1]
            theta2 = math.acos(cos_theta2) - math.atan2(z_eff, r)

            # Elbow angle
            cos_theta3 = (l2**2 + l3**2 - d**2) / (2 * l2 * l3)
            cos_theta3 = max(-1, min(1, cos_theta3))  # Clamp to [-1, 1]
            theta3 = math.pi - math.acos(cos_theta3)

            # Remaining joints (simplified)
            theta4 = 0.0
            theta5 = 0.0
            theta6 = 0.0

            # Create solution
            solution = np.array([theta1, theta2, theta3, theta4, theta5, theta6])

            return solution

        except Exception as e:
            self.get_logger().error(f'Inverse kinematics error: {str(e)}')
            return None

    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion"""
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                qw = (R[2, 1] - R[1, 2]) / s
                qx = 0.25 * s
                qy = (R[0, 1] + R[1, 0]) / s
                qz = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                qw = (R[0, 2] - R[2, 0]) / s
                qx = (R[0, 1] + R[1, 0]) / s
                qy = 0.25 * s
                qz = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                qw = (R[1, 0] - R[0, 1]) / s
                qx = (R[0, 2] + R[2, 0]) / s
                qy = (R[1, 2] + R[2, 1]) / s
                qz = 0.25 * s

        return [qx, qy, qz, qw]

    def solve_ik_for_pose(self, target_pose):
        """Solve inverse kinematics and publish solution"""
        ik_solution = self.inverse_kinematics(target_pose)

        if ik_solution is not None:
            # Publish solution
            solution_msg = Float32MultiArray()
            solution_msg.data = ik_solution.tolist()
            self.ik_solution_pub.publish(solution_msg)

            return ik_solution
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = ManipulationKinematics()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down manipulation kinematics...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Grasping and Grasp Planning

### Grasp Planning Algorithms

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import math
import threading
from scipy.spatial import KDTree

class GraspPlanner(Node):
    def __init__(self):
        super().__init__('grasp_planner')

        # Subscribe to object information
        self.object_sub = self.create_subscription(
            String,
            '/detected_object',
            self.object_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/object_pointcloud',
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.grasp_poses_pub = self.create_publisher(Pose, '/grasp_pose', 10)
        self.grasp_candidates_pub = self.create_publisher(MarkerArray, '/grasp_candidates', 10)

        # CV Bridge
        self.bridge = CvBridge()

        # Grasp planning parameters
        self.grasp_approach_distance = 0.1  # meters
        self.grasp_depth = 0.05  # meters
        self.grasp_width = 0.1  # meters
        self.min_grasp_quality = 0.7

        # Object information
        self.object_info = None
        self.object_points = None

        # Threading
        self.grasp_lock = threading.Lock()

        self.get_logger().info('Grasp Planner initialized')

    def object_callback(self, msg):
        """Process detected object information"""
        try:
            # Parse object information from message
            object_data = eval(msg.data)  # In practice, use proper serialization
            self.object_info = object_data

            # Plan grasps for the object
            self.plan_grasps()

        except Exception as e:
            self.get_logger().error(f'Object callback error: {str(e)}')

    def pointcloud_callback(self, msg):
        """Process object point cloud"""
        try:
            # Convert PointCloud2 to numpy array (simplified)
            # In practice, use sensor_msgs_py.point_cloud2.read_points
            self.object_points = self.pointcloud_to_array(msg)
            self.get_logger().info(f'Received object point cloud with {len(self.object_points) if self.object_points is not None else 0} points')

        except Exception as e:
            self.get_logger().error(f'Point cloud callback error: {str(e)}')

    def pointcloud_to_array(self, pointcloud_msg):
        """Convert PointCloud2 message to numpy array"""
        # This is a simplified version
        # In practice, use sensor_msgs_py.point_cloud2.read_points
        try:
            # For simulation, return random points
            # In real implementation, extract x, y, z coordinates
            return np.random.rand(100, 3) * 0.5  # Random points for simulation
        except:
            return None

    def plan_grasps(self):
        """Plan possible grasp poses for the object"""
        if self.object_info is None:
            return

        with self.grasp_lock:
            object_center = self.calculate_object_center()
            object_dimensions = self.calculate_object_dimensions()

            if object_center is not None and object_dimensions is not None:
                # Generate grasp candidates
                grasp_candidates = self.generate_grasp_candidates(object_center, object_dimensions)

                # Evaluate grasp quality
                high_quality_grasps = self.evaluate_grasps(grasp_candidates)

                if high_quality_grasps:
                    # Select the best grasp
                    best_grasp = self.select_best_grasp(high_quality_grasps)

                    # Publish the best grasp
                    self.grasp_poses_pub.publish(best_grasp)

                    # Publish visualization markers
                    self.publish_grasp_candidates(high_quality_grasps)

                    self.get_logger().info(f'Published best grasp for {self.object_info.get("label", "unknown")}')

    def calculate_object_center(self):
        """Calculate object center from point cloud"""
        if self.object_points is not None and len(self.object_points) > 0:
            center = np.mean(self.object_points, axis=0)
            return center
        elif self.object_info:
            # If point cloud not available, use bounding box center
            bbox = self.object_info.get('bbox', [0, 0, 0, 0])
            center_x = (bbox[0] + bbox[2]) / 2.0
            center_y = (bbox[1] + bbox[3]) / 2.0
            # Depth would come from other sensors
            return np.array([center_x, center_y, 0.5])

        return None

    def calculate_object_dimensions(self):
        """Calculate object dimensions from point cloud"""
        if self.object_points is not None and len(self.object_points) > 0:
            min_vals = np.min(self.object_points, axis=0)
            max_vals = np.max(self.object_points, axis=0)
            dimensions = max_vals - min_vals
            return dimensions
        elif self.object_info:
            # If point cloud not available, use bounding box dimensions
            bbox = self.object_info.get('bbox', [0, 0, 1, 1])
            width = bbox[2] - bbox[0]
            height = bbox[3] - bbox[1]
            depth = 0.1  # Default depth if not known
            return np.array([width, height, depth])

        return None

    def generate_grasp_candidates(self, object_center, object_dimensions):
        """Generate potential grasp poses around the object"""
        candidates = []

        # Generate grasps from different directions
        directions = [
            [1, 0, 0],   # Right
            [-1, 0, 0],  # Left
            [0, 1, 0],   # Front
            [0, -1, 0],  # Back
            [0, 0, 1],   # Top
            [0, 0, -1]   # Bottom
        ]

        for direction in directions:
            # Calculate grasp position
            grasp_pos = object_center + np.array(direction) * self.grasp_approach_distance

            # Calculate grasp orientation based on approach direction
            grasp_orientation = self.calculate_grasp_orientation(direction)

            # Create grasp pose
            grasp_pose = Pose()
            grasp_pose.position.x = float(grasp_pos[0])
            grasp_pose.position.y = float(grasp_pos[1])
            grasp_pose.position.z = float(grasp_pos[2])
            grasp_pose.orientation.x = float(grasp_orientation[0])
            grasp_pose.orientation.y = float(grasp_orientation[1])
            grasp_pose.orientation.z = float(grasp_orientation[2])
            grasp_pose.orientation.w = float(grasp_orientation[3])

            candidates.append(grasp_pose)

        return candidates

    def calculate_grasp_orientation(self, approach_direction):
        """Calculate grasp orientation based on approach direction"""
        # Normalize approach direction
        approach = np.array(approach_direction)
        approach = approach / np.linalg.norm(approach)

        # Calculate orientation that aligns gripper with approach direction
        # This is a simplified calculation
        if np.allclose(approach, [0, 0, 1]):  # Top-down grasp
            # Grasp from above, gripper aligned with Z-axis
            rotation = R.from_rotvec([0, 0, 0])
        elif np.allclose(approach, [1, 0, 0]):  # Side grasp along X
            rotation = R.from_rotvec([0, math.pi/2, 0])
        elif np.allclose(approach, [0, 1, 0]):  # Side grasp along Y
            rotation = R.from_rotvec([0, 0, -math.pi/2])
        else:
            # General case - align Z-axis of gripper with approach direction
            z_axis = np.array([0, 0, 1])
            if not np.allclose(approach, z_axis) and not np.allclose(approach, -z_axis):
                # Calculate rotation to align Z with approach
                rotation_axis = np.cross(z_axis, approach)
                rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
                angle = math.acos(np.dot(z_axis, approach))
                rotation = R.from_rotvec(rotation_axis * angle)
            else:
                rotation = R.from_rotvec([0, 0, 0])

        return rotation.as_quat()

    def evaluate_grasps(self, grasp_candidates):
        """Evaluate grasp quality for each candidate"""
        evaluated_grasps = []

        for grasp_pose in grasp_candidates:
            quality = self.evaluate_grasp_quality(grasp_pose)
            if quality >= self.min_grasp_quality:
                evaluated_grasps.append((grasp_pose, quality))

        # Sort by quality
        evaluated_grasps.sort(key=lambda x: x[1], reverse=True)

        return evaluated_grasps

    def evaluate_grasp_quality(self, grasp_pose):
        """Evaluate the quality of a grasp pose"""
        # In a real system, this would use more sophisticated metrics
        # For simulation, use simple heuristics

        # Check if grasp is in reach
        distance_from_base = math.sqrt(
            grasp_pose.position.x**2 +
            grasp_pose.position.y**2 +
            grasp_pose.position.z**2
        )

        if distance_from_base > 1.0:  # Beyond typical reach
            return 0.0

        # Check if grasp is stable (simplified)
        # For a top-down grasp, check if object is stable
        approach_direction = np.array([
            grasp_pose.position.x,
            grasp_pose.position.y,
            grasp_pose.position.z
        ])
        approach_direction = approach_direction / np.linalg.norm(approach_direction)

        # Stability based on approach direction
        stability = abs(approach_direction[2])  # Z component for top-down stability

        # Size consideration - larger objects might be harder to grasp
        if self.object_info:
            size_factor = min(1.0, 0.2 / self.object_info.get('size', 0.1))
        else:
            size_factor = 1.0

        quality = stability * size_factor

        return min(1.0, max(0.0, quality))

    def select_best_grasp(self, evaluated_grasps):
        """Select the best grasp from evaluated candidates"""
        if evaluated_grasps:
            return evaluated_grasps[0][0]  # Return the pose with highest quality
        else:
            return None

    def publish_grasp_candidates(self, evaluated_grasps):
        """Publish visualization markers for grasp candidates"""
        marker_array = MarkerArray()

        for i, (grasp_pose, quality) in enumerate(evaluated_grasps):
            # Create marker for grasp pose
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "grasp_candidates"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            # Set position and orientation
            marker.pose = grasp_pose

            # Set scale (length and width of arrow)
            marker.scale.x = 0.1  # Arrow shaft length
            marker.scale.y = 0.01  # Arrow shaft diameter
            marker.scale.z = 0.02  # Arrow head diameter

            # Set color based on quality
            marker.color.r = 1.0 - quality  # Red decreases with quality
            marker.color.g = quality      # Green increases with quality
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.grasp_candidates_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = GraspPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down grasp planner...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Manipulation Planning and Execution

### Manipulation Planning Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import math
import threading
import time

class ManipulationPlanner(Node):
    def __init__(self):
        super().__init__('manipulation_planner')

        # Subscribe to object and grasp information
        self.object_sub = self.create_subscription(
            String,
            '/object_to_manipulate',
            self.object_callback,
            10
        )

        self.grasp_sub = self.create_subscription(
            Pose,
            '/selected_grasp',
            self.grasp_callback,
            10
        )

        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.manipulation_status_pub = self.create_publisher(String, '/manipulation_status', 10)

        # Action server for manipulation tasks
        self._action_server = ActionServer(
            self,
            ManipulationAction,
            'manipulate_object',
            self.execute_manipulation_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Manipulation state
        self.current_object = None
        self.selected_grasp = None
        self.robot_joints = np.zeros(6)
        self.manipulation_active = False

        # Manipulation parameters
        self.approach_distance = 0.1
        self.lift_height = 0.2
        self.place_height = 0.1

        # Threading
        self.manipulation_lock = threading.Lock()

        self.get_logger().info('Manipulation Planner initialized')

    def object_callback(self, msg):
        """Process object to manipulate"""
        try:
            self.current_object = eval(msg.data)  # In practice, use proper deserialization
            self.get_logger().info(f'New object to manipulate: {self.current_object.get("label", "unknown")}')
        except Exception as e:
            self.get_logger().error(f'Object callback error: {str(e)}')

    def grasp_callback(self, msg):
        """Process selected grasp pose"""
        self.selected_grasp = msg
        self.get_logger().info('Received selected grasp pose')

    def goal_callback(self, goal_request):
        """Handle manipulation goal request"""
        self.get_logger().info(f'Received manipulation goal: {goal_request.action}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle manipulation goal cancellation"""
        self.get_logger().info('Received manipulation goal cancellation')
        return CancelResponse.ACCEPT

    async def execute_manipulation_callback(self, goal_handle):
        """Execute manipulation task"""
        self.get_logger().info('Executing manipulation task')

        feedback_msg = ManipulationAction.Feedback()
        result = ManipulationAction.Result()

        with self.manipulation_lock:
            try:
                action = goal_handle.request.action
                target_pose = goal_handle.request.target_pose

                if action == "pick":
                    success = await self.execute_pick_task(target_pose, feedback_msg)
                elif action == "place":
                    success = await self.execute_place_task(target_pose, feedback_msg)
                elif action == "move":
                    success = await self.execute_move_task(target_pose, feedback_msg)
                else:
                    self.get_logger().error(f'Unknown manipulation action: {action}')
                    success = False

                if success:
                    goal_handle.succeed()
                    result.success = True
                    result.message = f'Successfully completed {action} action'
                else:
                    goal_handle.abort()
                    result.success = False
                    result.message = f'Failed to complete {action} action'

            except Exception as e:
                self.get_logger().error(f'Manipulation execution error: {str(e)}')
                goal_handle.abort()
                result.success = False
                result.message = f'Execution error: {str(e)}'

        return result

    async def execute_pick_task(self, target_pose, feedback_msg):
        """Execute pick task"""
        self.get_logger().info('Executing pick task')

        # Step 1: Approach the object
        approach_pose = self.calculate_approach_pose(target_pose)
        if not await self.move_to_pose(approach_pose, feedback_msg, "approaching_object"):
            return False

        # Step 2: Move to grasp position
        if not await self.move_to_pose(target_pose, feedback_msg, "moving_to_grasp"):
            return False

        # Step 3: Close gripper
        if not await self.close_gripper(feedback_msg):
            return False

        # Step 4: Lift object
        lift_pose = self.calculate_lift_pose(target_pose)
        if not await self.move_to_pose(lift_pose, feedback_msg, "lifting_object"):
            return False

        self.get_logger().info('Pick task completed successfully')
        return True

    async def execute_place_task(self, target_pose, feedback_msg):
        """Execute place task"""
        self.get_logger().info('Executing place task')

        # Step 1: Approach placement location
        approach_pose = self.calculate_approach_pose(target_pose)
        if not await self.move_to_pose(approach_pose, feedback_msg, "approaching_placement"):
            return False

        # Step 2: Move to placement position
        if not await self.move_to_pose(target_pose, feedback_msg, "moving_to_placement"):
            return False

        # Step 3: Open gripper
        if not await self.open_gripper(feedback_msg):
            return False

        # Step 4: Retreat
        retreat_pose = self.calculate_retreat_pose(target_pose)
        if not await self.move_to_pose(retreat_pose, feedback_msg, "retreating"):
            return False

        self.get_logger().info('Place task completed successfully')
        return True

    async def execute_move_task(self, target_pose, feedback_msg):
        """Execute move task"""
        self.get_logger().info('Executing move task')

        # Simply move to the target pose
        if not await self.move_to_pose(target_pose, feedback_msg, "moving_to_target"):
            return False

        self.get_logger().info('Move task completed successfully')
        return True

    def calculate_approach_pose(self, target_pose):
        """Calculate approach pose before grasping"""
        approach_pose = Pose()
        approach_pose.position = target_pose.position
        approach_pose.orientation = target_pose.orientation

        # Move back slightly along approach direction
        approach_direction = self.calculate_approach_vector(target_pose)
        approach_pose.position.x -= approach_direction[0] * self.approach_distance
        approach_pose.position.y -= approach_direction[1] * self.approach_distance
        approach_pose.position.z -= approach_direction[2] * self.approach_distance

        return approach_pose

    def calculate_lift_pose(self, grasp_pose):
        """Calculate lift pose after grasping"""
        lift_pose = Pose()
        lift_pose.position = grasp_pose.position
        lift_pose.orientation = grasp_pose.orientation

        # Move up
        lift_pose.position.z += self.lift_height

        return lift_pose

    def calculate_retreat_pose(self, place_pose):
        """Calculate retreat pose after placing"""
        retreat_pose = Pose()
        retreat_pose.position = place_pose.position
        retreat_pose.orientation = place_pose.orientation

        # Move back slightly
        retreat_pose.position.z += self.approach_distance

        return retreat_pose

    def calculate_approach_vector(self, target_pose):
        """Calculate approach vector from orientation"""
        # Extract approach direction from orientation quaternion
        # This is simplified - in practice, use the orientation to determine approach direction
        return [0, 0, -1]  # Default: approach from above

    async def move_to_pose(self, target_pose, feedback_msg, stage):
        """Move end effector to target pose"""
        try:
            # In a real system, this would solve IK and send joint commands
            # For simulation, we'll just publish a status update

            # Publish feedback
            feedback_msg.current_stage = stage
            feedback_msg.progress = 0.0

            # Simulate movement (in real system, this would be actual movement)
            for i in range(10):  # Simulate 10 steps
                feedback_msg.progress = (i + 1) / 10.0
                goal_handle.publish_feedback(feedback_msg)
                await rclpy.sleep(0.1)  # Simulate movement time

            self.get_logger().info(f'Moved to pose for {stage}')
            return True

        except Exception as e:
            self.get_logger().error(f'Move to pose error: {str(e)}')
            return False

    async def close_gripper(self, feedback_msg):
        """Close gripper to grasp object"""
        try:
            # Simulate gripper closing
            self.get_logger().info('Closing gripper')

            # Publish gripper command (in real system)
            # gripper_cmd_msg = Float64()
            # gripper_cmd_msg.data = 0.0  # Closed position
            # self.gripper_pub.publish(gripper_cmd_msg)

            # Simulate gripper closing time
            await rclpy.sleep(0.5)

            return True

        except Exception as e:
            self.get_logger().error(f'Close gripper error: {str(e)}')
            return False

    async def open_gripper(self, feedback_msg):
        """Open gripper to release object"""
        try:
            # Simulate gripper opening
            self.get_logger().info('Opening gripper')

            # Publish gripper command (in real system)
            # gripper_cmd_msg = Float64()
            # gripper_cmd_msg.data = 1.0  # Open position
            # self.gripper_pub.publish(gripper_cmd_msg)

            # Simulate gripper opening time
            await rclpy.sleep(0.5)

            return True

        except Exception as e:
            self.get_logger().error(f'Open gripper error: {str(e)}')
            return False

    def publish_manipulation_status(self, status):
        """Publish manipulation status"""
        status_msg = String()
        status_msg.data = status
        self.manipulation_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ManipulationPlanner()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        print("Shutting down manipulation planner...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Force Control and Tactile Feedback

### Force-Controlled Manipulation

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import numpy as np
import math
import threading

class ForceControlledManipulator(Node):
    def __init__(self):
        super().__init__('force_controlled_manipulator')

        # Subscribe to force/torque sensor
        self.force_sub = self.create_subscription(
            WrenchStamped,
            '/wrench',
            self.force_callback,
            10
        )

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers
        self.force_cmd_pub = self.create_publisher(Wrench, '/force_command', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.contact_status_pub = self.create_publisher(Float32, '/contact_force', 10)

        # Force control parameters
        self.desired_force = np.array([0.0, 0.0, 5.0])  # 5N downward force
        self.force_tolerance = 0.5  # 0.5N tolerance
        self.stiffness = 100.0  # N/m stiffness
        self.damping = 10.0     # N*s/m damping

        # Current state
        self.current_force = np.array([0.0, 0.0, 0.0])
        self.current_joints = np.zeros(6)
        self.contact_threshold = 2.0  # N

        # Force control state
        self.force_control_active = False
        self.impedance_control_active = False

        # Threading
        self.force_control_lock = threading.Lock()

        self.get_logger().info('Force-Controlled Manipulator initialized')

    def force_callback(self, msg):
        """Process force/torque sensor data"""
        with self.force_control_lock:
            self.current_force = np.array([
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z
            ])

            # Publish contact status
            contact_force = math.sqrt(sum(f**2 for f in self.current_force))
            contact_msg = Float32()
            contact_msg.data = contact_force
            self.contact_status_pub.publish(contact_msg)

            # Check if contact is detected
            if contact_force > self.contact_threshold:
                self.get_logger().info(f'Contact detected: {contact_force:.2f}N')

            # Execute force control if active
            if self.force_control_active:
                self.execute_force_control()

    def joint_state_callback(self, msg):
        """Update joint states"""
        if len(msg.position) >= 6:
            self.current_joints = np.array(msg.position[:6])

    def execute_force_control(self):
        """Execute force control algorithm"""
        if not self.force_control_active:
            return

        # Calculate force error
        force_error = self.desired_force - self.current_force

        # Simple PID-based force control
        force_command = self.calculate_force_command(force_error)

        # Publish force command
        wrench_msg = Wrench()
        wrench_msg.force.x = float(force_command[0])
        wrench_msg.force.y = float(force_command[1])
        wrench_msg.force.z = float(force_command[2])
        # Torque commands would go here if needed
        self.force_cmd_pub.publish(wrench_msg)

        # Log force control status
        self.get_logger().info(f'Force control - Desired: {self.desired_force}, Actual: {self.current_force}, Error: {force_error}')

    def calculate_force_command(self, force_error):
        """Calculate force command based on error"""
        # Simple proportional control
        kp = 1.0  # Proportional gain
        force_command = kp * force_error

        # Limit command magnitude
        max_force = 10.0  # N
        force_magnitude = np.linalg.norm(force_command)
        if force_magnitude > max_force:
            force_command = (force_command / force_magnitude) * max_force

        return force_command

    def execute_impedance_control(self, desired_position, desired_stiffness=None, desired_damping=None):
        """Execute impedance control"""
        if not self.impedance_control_active:
            return

        # Calculate position error
        current_position = self.get_end_effector_position()
        position_error = desired_position - current_position

        # Calculate desired force based on impedance model
        if desired_stiffness is not None:
            stiffness = desired_stiffness
        else:
            stiffness = self.stiffness

        if desired_damping is not None:
            damping = desired_damping
        else:
            damping = self.damping

        # Impedance model: F = K(x_d - x) + D(v_d - v)
        # For position control: F = K(x_d - x)
        desired_force = stiffness * position_error

        # Apply force command
        wrench_msg = Wrench()
        wrench_msg.force.x = float(desired_force[0])
        wrench_msg.force.y = float(desired_force[1])
        wrench_msg.force.z = float(desired_force[2])
        self.force_cmd_pub.publish(wrench_msg)

    def get_end_effector_position(self):
        """Get end effector position (simplified)"""
        # In a real system, this would use forward kinematics
        # For simulation, return current joint-based position
        # This is a simplified approximation
        return np.array([self.current_joints[0], self.current_joints[1], self.current_joints[2]])

    def start_force_control(self, desired_force):
        """Start force control with desired force"""
        with self.force_control_lock:
            self.desired_force = np.array(desired_force)
            self.force_control_active = True
            self.get_logger().info(f'Started force control with desired force: {self.desired_force}')

    def stop_force_control(self):
        """Stop force control"""
        with self.force_control_lock:
            self.force_control_active = False
            self.get_logger().info('Stopped force control')

    def start_impedance_control(self):
        """Start impedance control"""
        self.impedance_control_active = True
        self.get_logger().info('Started impedance control')

    def stop_impedance_control(self):
        """Stop impedance control"""
        self.impedance_control_active = False
        self.get_logger().info('Stopped impedance control')

def main(args=None):
    rclpy.init(args=args)
    node = ForceControlledManipulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down force-controlled manipulator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Multi-Fingered Hand Control

### Multi-Fingered Grasp Control

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np
import math
import threading

class MultiFingeredHandController(Node):
    def __init__(self):
        super().__init__('multi_fingered_hand_controller')

        # Publishers
        self.hand_cmd_pub = self.create_publisher(Float64MultiArray, '/hand_commands', 10)
        self.gripper_cmd_pub = self.create_publisher(JointState, '/gripper_commands', 10)
        self.grasp_status_pub = self.create_publisher(String, '/grasp_status', 10)

        # Subscribe to grasp planning
        self.grasp_plan_sub = self.create_subscription(
            String,
            '/grasp_plan',
            self.grasp_plan_callback,
            10
        )

        # Hand configuration (example: 3-finger hand)
        self.num_fingers = 3
        self.finger_joints = 3  # joints per finger
        self.total_joints = self.num_fingers * self.finger_joints

        # Current hand state
        self.current_hand_positions = np.zeros(self.total_joints)
        self.current_hand_velocities = np.zeros(self.total_joints)
        self.current_hand_efforts = np.zeros(self.total_joints)

        # Grasp parameters
        self.grasp_types = {
            'power': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],  # Strong grasp
            'precision': [0.3, 0.3, 0.3, 0.3, 0.3, 0.3],  # Gentle grasp
            'cylindrical': [0.7, 0.7, 0.7, 0.7, 0.7, 0.7],  # Cylindrical grasp
            'spherical': [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]   # Spherical grasp
        }

        # Threading
        self.hand_control_lock = threading.Lock()

        self.get_logger().info('Multi-Fingered Hand Controller initialized')

    def grasp_plan_callback(self, msg):
        """Process grasp plan"""
        try:
            grasp_plan = eval(msg.data)  # In practice, use proper deserialization
            grasp_type = grasp_plan.get('type', 'power')
            object_size = grasp_plan.get('size', 0.1)

            # Execute grasp based on plan
            self.execute_grasp(grasp_type, object_size)

        except Exception as e:
            self.get_logger().error(f'Grasp plan callback error: {str(e)}')

    def execute_grasp(self, grasp_type, object_size):
        """Execute grasp based on type and object size"""
        with self.hand_control_lock:
            if grasp_type in self.grasp_types:
                # Get basic grasp configuration
                base_positions = np.array(self.grasp_types[grasp_type])

                # Adjust for object size
                size_factor = min(1.0, object_size / 0.1)  # Normalize to 10cm object
                adjusted_positions = base_positions * (1.0 - size_factor * 0.3)  # Smaller objects need more closure

                # Publish hand commands
                self.publish_hand_commands(adjusted_positions)

                # Publish grasp status
                status_msg = String()
                status_msg.data = f"executing_{grasp_type}_grasp_size_{object_size:.3f}m"
                self.grasp_status_pub.publish(status_msg)

                self.get_logger().info(f'Executing {grasp_type} grasp for {object_size:.3f}m object')

    def publish_hand_commands(self, joint_positions):
        """Publish commands to multi-fingered hand"""
        # Create Float64MultiArray message
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joint_positions.tolist()
        self.hand_cmd_pub.publish(cmd_msg)

        # Also publish as JointState for compatibility
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f'finger_{i}_joint_{j}' for i in range(self.num_fingers) for j in range(self.finger_joints)]
        joint_state_msg.position = joint_positions.tolist()
        joint_state_msg.velocity = [0.0] * len(joint_positions)
        joint_state_msg.effort = [0.0] * len(joint_positions)
        self.gripper_cmd_pub.publish(joint_state_msg)

    def execute_predefined_grasp(self, grasp_name):
        """Execute a predefined grasp type"""
        if grasp_name in self.grasp_types:
            positions = np.array(self.grasp_types[grasp_name])
            self.publish_hand_commands(positions)
            self.get_logger().info(f'Executed {grasp_name} grasp')

    def execute_custom_grasp(self, finger_positions):
        """Execute custom grasp with specific finger positions"""
        if len(finger_positions) == self.total_joints:
            self.publish_hand_commands(np.array(finger_positions))
            self.get_logger().info('Executed custom grasp')

    def open_hand(self):
        """Open all fingers"""
        open_positions = np.zeros(self.total_joints)
        self.publish_hand_commands(open_positions)
        self.get_logger().info('Opened hand')

    def close_hand(self):
        """Close all fingers firmly"""
        close_positions = np.ones(self.total_joints) * 1.5  # Maximum closure
        self.publish_hand_commands(close_positions)
        self.get_logger().info('Closed hand firmly')

    def adjust_grasp_force(self, force_multiplier):
        """Adjust grasp force by scaling joint positions"""
        # This is a simplified approach - in reality, force control would be more complex
        current_positions = self.current_hand_positions.copy()
        adjusted_positions = current_positions * force_multiplier
        # Limit to valid joint ranges
        adjusted_positions = np.clip(adjusted_positions, 0.0, 1.5)
        self.publish_hand_commands(adjusted_positions)
        self.get_logger().info(f'Adjusted grasp force by {force_multiplier:.2f}x')

    def get_hand_status(self):
        """Get current hand status"""
        status = {
            'num_fingers': self.num_fingers,
            'joints_per_finger': self.finger_joints,
            'total_joints': self.total_joints,
            'current_positions': self.current_hand_positions.tolist(),
            'grasp_types': list(self.grasp_types.keys())
        }
        return status

def main(args=None):
    rclpy.init(args=args)
    node = MultiFingeredHandController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down multi-fingered hand controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Lab: Complete Manipulation System

In this lab, you'll integrate all manipulation components into a complete system.

### Step 1: Create the Manipulation System Launch File

Create `manipulation_system_launch.py`:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Manipulation system nodes
    kinematics = Node(
        package='ai_robo_learning',
        executable='manipulation_kinematics',
        name='manipulation_kinematics',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    grasp_planner = Node(
        package='ai_robo_learning',
        executable='grasp_planner',
        name='grasp_planner',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    manipulation_planner = Node(
        package='ai_robo_learning',
        executable='manipulation_planner',
        name='manipulation_planner',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    force_controller = Node(
        package='ai_robo_learning',
        executable='force_controlled_manipulator',
        name='force_controlled_manipulator',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    hand_controller = Node(
        package='ai_robo_learning',
        executable='multi_fingered_hand_controller',
        name='multi_fingered_hand_controller',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Return launch description
    ld = LaunchDescription()

    # Add all nodes
    ld.add_action(kinematics)
    ld.add_action(grasp_planner)
    ld.add_action(manipulation_planner)
    ld.add_action(force_controller)
    ld.add_action(hand_controller)

    return ld
```

### Step 2: Create the Complete Manipulation Node

Create `complete_manipulation_system.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState, WrenchStamped
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import math
import threading
import time
from enum import Enum

class ManipulationState(Enum):
    IDLE = 0
    APPROACHING = 1
    GRASPING = 2
    LIFTING = 3
    MOVING = 4
    PLACING = 5
    RETRACTING = 6
    ERROR = 7

class CompleteManipulationSystem(Node):
    def __init__(self):
        super().__init__('complete_manipulation_system')

        # Subscribe to various inputs
        self.object_sub = self.create_subscription(
            String,
            '/object_to_manipulate',
            self.object_callback,
            10
        )

        self.grasp_pose_sub = self.create_subscription(
            Pose,
            '/selected_grasp',
            self.grasp_pose_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.force_sub = self.create_subscription(
            WrenchStamped,
            '/wrench',
            self.force_callback,
            10
        )

        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.hand_cmd_pub = self.create_publisher(String, '/hand_commands', 10)
        self.status_pub = self.create_publisher(String, '/manipulation_status', 10)
        self.error_pub = self.create_publisher(String, '/manipulation_error', 10)

        # Action server for manipulation tasks
        self._action_server = ActionServer(
            self,
            ManipulationAction,
            'complete_manipulation',
            self.execute_manipulation_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # System state
        self.current_object = None
        self.selected_grasp = None
        self.current_joints = np.zeros(6)
        self.current_force = np.array([0.0, 0.0, 0.0])
        self.manipulation_state = ManipulationState.IDLE

        # Manipulation parameters
        self.approach_distance = 0.1
        self.lift_height = 0.2
        self.retract_distance = 0.1
        self.max_force_threshold = 20.0  # N

        # Threading
        self.manipulation_lock = threading.Lock()
        self.force_monitor_thread = threading.Thread(target=self.force_monitor, daemon=True)
        self.force_monitor_thread.start()

        # Safety parameters
        self.safety_enabled = True
        self.collision_threshold = 0.05  # m

        self.get_logger().info('Complete Manipulation System initialized')

    def object_callback(self, msg):
        """Process object information"""
        try:
            self.current_object = eval(msg.data)  # In practice, use proper deserialization
            self.get_logger().info(f'New object: {self.current_object.get("label", "unknown")}')
        except Exception as e:
            self.get_logger().error(f'Object callback error: {str(e)}')

    def grasp_pose_callback(self, msg):
        """Process grasp pose"""
        self.selected_grasp = msg
        self.get_logger().info('Received grasp pose')

    def joint_state_callback(self, msg):
        """Update joint states"""
        if len(msg.position) >= 6:
            self.current_joints = np.array(msg.position[:6])

    def force_callback(self, msg):
        """Update force sensor data"""
        self.current_force = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ])

    def force_monitor(self):
        """Monitor forces and trigger safety responses"""
        while rclpy.ok():
            if self.safety_enabled:
                force_magnitude = np.linalg.norm(self.current_force)

                if force_magnitude > self.max_force_threshold:
                    self.trigger_safety_response(f'Force limit exceeded: {force_magnitude:.2f}N > {self.max_force_threshold}N')

            time.sleep(0.01)  # 100Hz monitoring

    def goal_callback(self, goal_request):
        """Handle manipulation goal request"""
        self.get_logger().info(f'Received manipulation goal: {goal_request.action}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle manipulation goal cancellation"""
        self.get_logger().info('Received manipulation goal cancellation')
        return CancelResponse.ACCEPT

    async def execute_manipulation_callback(self, goal_handle):
        """Execute complete manipulation task"""
        self.get_logger().info('Executing complete manipulation task')

        feedback_msg = ManipulationAction.Feedback()
        result = ManipulationAction.Result()

        with self.manipulation_lock:
            try:
                action = goal_handle.request.action
                target_pose = goal_handle.request.target_pose

                if action == "pick_and_place":
                    success = await self.execute_pick_and_place_task(target_pose, feedback_msg)
                elif action == "grasp":
                    success = await self.execute_grasp_task(target_pose, feedback_msg)
                elif action == "move_object":
                    success = await self.execute_move_object_task(target_pose, feedback_msg)
                else:
                    self.get_logger().error(f'Unknown manipulation action: {action}')
                    success = False

                if success:
                    goal_handle.succeed()
                    result.success = True
                    result.message = f'Successfully completed {action} action'
                else:
                    goal_handle.abort()
                    result.success = False
                    result.message = f'Failed to complete {action} action'

            except Exception as e:
                self.get_logger().error(f'Manipulation execution error: {str(e)}')
                goal_handle.abort()
                result.success = False
                result.message = f'Execution error: {str(e)}'

        return result

    async def execute_pick_and_place_task(self, target_pose, feedback_msg):
        """Execute complete pick and place task"""
        self.get_logger().info('Executing pick and place task')

        # Step 1: Approach object
        self.manipulation_state = ManipulationState.APPROACHING
        feedback_msg.current_stage = "approaching_object"
        feedback_msg.progress = 0.0
        goal_handle.publish_feedback(feedback_msg)

        approach_pose = self.calculate_approach_pose(self.selected_grasp)
        if not await self.move_to_pose(approach_pose, feedback_msg, "approaching_object"):
            return False

        # Step 2: Grasp object
        self.manipulation_state = ManipulationState.GRASPING
        feedback_msg.current_stage = "grasping_object"
        goal_handle.publish_feedback(feedback_msg)

        if not await self.execute_grasp(feedback_msg):
            return False

        # Step 3: Lift object
        self.manipulation_state = ManipulationState.LIFTING
        feedback_msg.current_stage = "lifting_object"
        goal_handle.publish_feedback(feedback_msg)

        lift_pose = self.calculate_lift_pose(self.selected_grasp)
        if not await self.move_to_pose(lift_pose, feedback_msg, "lifting_object"):
            return False

        # Step 4: Move to target location
        self.manipulation_state = ManipulationState.MOVING
        feedback_msg.current_stage = "moving_to_target"
        goal_handle.publish_feedback(feedback_msg)

        if not await self.move_to_pose(target_pose, feedback_msg, "moving_to_target"):
            return False

        # Step 5: Place object
        self.manipulation_state = ManipulationState.PLACING
        feedback_msg.current_stage = "placing_object"
        goal_handle.publish_feedback(feedback_msg)

        if not await self.execute_place(feedback_msg):
            return False

        # Step 6: Retract
        self.manipulation_state = ManipulationState.RETRACTING
        feedback_msg.current_stage = "retracting"
        goal_handle.publish_feedback(feedback_msg)

        retract_pose = self.calculate_retreat_pose(target_pose)
        if not await self.move_to_pose(retract_pose, feedback_msg, "retracting"):
            return False

        self.manipulation_state = ManipulationState.IDLE
        self.get_logger().info('Pick and place task completed successfully')
        return True

    async def execute_grasp_task(self, target_pose, feedback_msg):
        """Execute grasp task"""
        self.get_logger().info('Executing grasp task')

        # Approach
        approach_pose = self.calculate_approach_pose(target_pose)
        if not await self.move_to_pose(approach_pose, feedback_msg, "approaching"):
            return False

        # Grasp
        if not await self.execute_grasp(feedback_msg):
            return False

        self.get_logger().info('Grasp task completed successfully')
        return True

    async def execute_move_object_task(self, target_pose, feedback_msg):
        """Execute move object task"""
        self.get_logger().info('Executing move object task')

        # Move to target pose
        if not await self.move_to_pose(target_pose, feedback_msg, "moving_object"):
            return False

        self.get_logger().info('Move object task completed successfully')
        return True

    def calculate_approach_pose(self, target_pose):
        """Calculate approach pose"""
        approach_pose = Pose()
        approach_pose.position = target_pose.position
        approach_pose.orientation = target_pose.orientation

        # Move back along approach direction
        approach_direction = self.calculate_approach_vector(target_pose)
        approach_pose.position.x -= approach_direction[0] * self.approach_distance
        approach_pose.position.y -= approach_direction[1] * self.approach_distance
        approach_pose.position.z -= approach_direction[2] * self.approach_distance

        return approach_pose

    def calculate_lift_pose(self, grasp_pose):
        """Calculate lift pose"""
        lift_pose = Pose()
        lift_pose.position = grasp_pose.position
        lift_pose.orientation = grasp_pose.orientation

        # Move up
        lift_pose.position.z += self.lift_height

        return lift_pose

    def calculate_retreat_pose(self, place_pose):
        """Calculate retreat pose"""
        retreat_pose = Pose()
        retreat_pose.position = place_pose.position
        retreat_pose.orientation = place_pose.orientation

        # Move up
        retreat_pose.position.z += self.retract_distance

        return retreat_pose

    def calculate_approach_vector(self, target_pose):
        """Calculate approach vector"""
        # For top-down grasp, approach from above
        return [0, 0, -1]

    async def move_to_pose(self, target_pose, feedback_msg, stage):
        """Move end effector to target pose"""
        try:
            # In a real system, this would solve IK and move the arm
            # For simulation, just log the movement
            self.get_logger().info(f'Moving to pose for {stage}')

            # Simulate movement progress
            for i in range(10):
                feedback_msg.progress = (i + 1) / 10.0
                goal_handle.publish_feedback(feedback_msg)
                await rclpy.sleep(0.1)

            return True

        except Exception as e:
            self.get_logger().error(f'Move to pose error: {str(e)}')
            return False

    async def execute_grasp(self, feedback_msg):
        """Execute grasping action"""
        try:
            self.get_logger().info('Executing grasp')

            # Publish hand command to close gripper
            hand_cmd = String()
            hand_cmd.data = "close_gripper"
            self.hand_cmd_pub.publish(hand_cmd)

            # Simulate grasp time
            await rclpy.sleep(0.5)

            # Check if grasp was successful (simplified)
            force_magnitude = np.linalg.norm(self.current_force)
            if force_magnitude > 1.0:  # Some contact detected
                self.get_logger().info('Grasp successful')
                return True
            else:
                self.get_logger().warn('Grasp may have failed - no contact detected')
                return False

        except Exception as e:
            self.get_logger().error(f'Grasp execution error: {str(e)}')
            return False

    async def execute_place(self, feedback_msg):
        """Execute placing action"""
        try:
            self.get_logger().info('Executing place')

            # Publish hand command to open gripper
            hand_cmd = String()
            hand_cmd.data = "open_gripper"
            self.hand_cmd_pub.publish(hand_cmd)

            # Simulate release time
            await rclpy.sleep(0.5)

            self.get_logger().info('Place completed')
            return True

        except Exception as e:
            self.get_logger().error(f'Place execution error: {str(e)}')
            return False

    def trigger_safety_response(self, error_msg):
        """Trigger safety response for excessive forces"""
        self.get_logger().error(f'Safety response triggered: {error_msg}')

        # Publish error
        error_message = String()
        error_message.data = error_msg
        self.error_pub.publish(error_message)

        # Stop all motion (in a real system, send stop commands)
        self.manipulation_state = ManipulationState.ERROR

        # Publish emergency stop (simplified)
        stop_cmd = JointState()
        stop_cmd.position = [0.0] * len(self.current_joints)
        self.joint_cmd_pub.publish(stop_cmd)

    def publish_status(self, status):
        """Publish manipulation status"""
        status_msg = String()
        status_msg.data = f"{self.manipulation_state.name}:{status}"
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CompleteManipulationSystem()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        print("Shutting down complete manipulation system...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Test the Manipulation System

1. Make sure you have the required dependencies:
```bash
pip3 install numpy scipy
```

2. Run the complete manipulation system:
```bash
python3 complete_manipulation_system.py
```

3. Test with manipulation commands:
```bash
# Send a manipulation goal using action interface
# Or use direct commands to test individual components
```

## Best Practices

1. **Safety First**: Always implement safety checks and force limits
2. **Force Control**: Use force feedback for compliant manipulation
3. **Grasp Planning**: Plan grasps considering object properties and robot capabilities
4. **Kinematics**: Ensure proper forward and inverse kinematics
5. **Calibration**: Calibrate sensors and actuators for accurate manipulation
6. **Validation**: Validate grasps before execution
7. **Error Handling**: Implement robust error recovery
8. **Testing**: Test extensively with various objects and scenarios

## Next Steps

After completing this chapter, you'll be ready to learn about the capstone project in Chapter 6, where you'll integrate all the components learned throughout the course into a complete humanoid robot system.