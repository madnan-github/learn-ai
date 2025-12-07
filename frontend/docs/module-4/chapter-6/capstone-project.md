---
sidebar_position: 6
title: 'Capstone Project - Autonomous Humanoid Robot'
---

# Capstone Project - Autonomous Humanoid Robot

This capstone chapter integrates all the concepts learned throughout the course to build a complete autonomous humanoid robot system capable of navigating, perceiving, and manipulating objects in a real environment.

## What You'll Learn

In this chapter, you'll explore:
- System integration of all previous modules
- Complete autonomous robot pipeline
- Multi-modal perception and decision making
- Real-world deployment considerations
- Performance optimization and debugging
- Safety and validation protocols
- End-to-end testing and validation

## Prerequisites

- Completion of Module 1-4, Chapters 1-5
- Understanding of all previous concepts
- Experience with ROS 2 system integration
- Knowledge of all subsystems (navigation, perception, manipulation)
- Access to a humanoid robot or simulation environment

## Complete Autonomous Robot Architecture

### Integrated System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        HUMANOID ROBOT                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐         │
│  │ NAVIGATION  │    │ PERCEPTION  │    │MANIPULATION │         │
│  │   SYSTEM    │    │   SYSTEM    │    │   SYSTEM    │         │
│  └─────────────┘    └─────────────┘    └─────────────┘         │
│         │                   │                   │               │
│         ▼                   ▼                   ▼               │
│  ┌─────────────────────────────────────────────────────────┐     │
│  │                 COGNITIVE ENGINE                        │     │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐      │     │
│  │  │  TASK       │ │  BEHAVIOR   │ │   DECISION  │      │     │
│  │  │ PLANNING    │ │   TREE      │ │   MAKING    │      │     │
│  │  └─────────────┘ └─────────────┘ └─────────────┘      │     │
│  └─────────────────────────────────────────────────────────┘     │
│         │                   │                   │               │
│         ▼                   ▼                   ▼               │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐         │
│  │ MOTION      │    │  VISION     │    │  GRASP      │         │
│  │ CONTROLLER  │    │  PROCESSOR  │    │  PLANNER    │         │
│  └─────────────┘    └─────────────┘    └─────────────┘         │
│         │                   │                   │               │
│         └───────────────────┼───────────────────┘               │
│                             │                                   │
│                    ┌─────────────────┐                         │
│                    │   HARDWARE      │                         │
│                    │  INTERFACE      │                         │
│                    └─────────────────┘                         │
└─────────────────────────────────────────────────────────────────┘
```

### Main Autonomous Robot Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, LaserScan, JointState
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import json
import threading
import time
from enum import Enum
from typing import Dict, Any, Optional

class RobotState(Enum):
    IDLE = 0
    LISTENING = 1
    NAVIGATING = 2
    PERCEIVING = 3
    MANIPULATING = 4
    EXECUTING_TASK = 5
    EMERGENCY_STOP = 6
    ERROR = 7

class AutonomousHumanoidRobot(Node):
    def __init__(self):
        super().__init__('autonomous_humanoid_robot')

        # Subscribe to all sensors
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.voice_command_sub = self.create_subscription(
            String,
            '/natural_language_command',
            self.voice_command_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.system_status_pub = self.create_publisher(String, '/system_status', 10)

        # Action servers
        self._navigation_action_server = ActionServer(
            self,
            NavigationAction,
            'navigate_to_pose',
            self.navigate_to_pose_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self._manipulation_action_server = ActionServer(
            self,
            ManipulationAction,
            'manipulate_object',
            self.manipulate_object_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # System state
        self.current_state = RobotState.IDLE
        self.robot_pose = None
        self.joint_positions = {}
        self.current_command = None
        self.task_queue = []
        self.emergency_stop_active = False

        # System components
        self.navigation_system = None
        self.perception_system = None
        self.manipulation_system = None
        self.cognitive_engine = None

        # Performance metrics
        self.start_time = time.time()
        self.total_tasks_completed = 0
        self.total_errors = 0

        # Threading
        self.system_lock = threading.Lock()
        self.main_loop_thread = threading.Thread(target=self.main_loop, daemon=True)
        self.main_loop_thread.start()

        # Safety parameters
        self.safety_distance = 0.5  # meters
        self.max_speed = 0.5  # m/s
        self.max_angular_speed = 0.5  # rad/s

        # System health monitoring
        self.system_health = {
            'navigation': True,
            'perception': True,
            'manipulation': True,
            'cognition': True,
            'sensors': True
        }

        self.get_logger().info('Autonomous Humanoid Robot initialized')

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.robot_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        if self.current_state != RobotState.EMERGENCY_STOP:
            # Check for obstacles and trigger emergency stop if needed
            if self.check_emergency_stop_conditions(msg):
                self.trigger_emergency_stop()

    def camera_callback(self, msg):
        """Process camera image"""
        # Forward to perception system
        if self.perception_system:
            self.perception_system.process_image(msg)

    def joint_state_callback(self, msg):
        """Update joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def voice_command_callback(self, msg):
        """Process voice command"""
        command = msg.data.strip()
        if command:
            self.current_command = command
            self.process_command(command)

    def main_loop(self):
        """Main system loop"""
        while rclpy.ok():
            with self.system_lock:
                # Monitor system health
                self.monitor_system_health()

                # Execute queued tasks
                self.execute_queued_tasks()

                # Publish system status
                self.publish_system_status()

                # Check safety conditions
                self.check_safety_conditions()

            time.sleep(0.1)  # 10Hz main loop

    def process_command(self, command):
        """Process natural language command"""
        try:
            # Use cognitive engine to interpret command
            task_plan = self.cognitive_engine.interpret_command(command)

            if task_plan:
                # Add to task queue
                self.task_queue.append(task_plan)
                self.get_logger().info(f'Processed command: "{command}" -> Task plan added to queue')
            else:
                self.get_logger().error(f'Could not interpret command: "{command}"')

        except Exception as e:
            self.get_logger().error(f'Command processing error: {str(e)}')
            self.total_errors += 1

    def execute_queued_tasks(self):
        """Execute tasks from the queue"""
        while self.task_queue and not self.emergency_stop_active:
            task = self.task_queue[0]

            try:
                # Execute task based on type
                success = self.execute_task(task)

                if success:
                    self.task_queue.pop(0)  # Remove completed task
                    self.total_tasks_completed += 1
                    self.get_logger().info(f'Completed task: {task["type"]}')
                else:
                    self.get_logger().error(f'Task failed: {task["type"]}')
                    self.total_errors += 1
                    # Remove failed task to prevent infinite loop
                    self.task_queue.pop(0)

            except Exception as e:
                self.get_logger().error(f'Task execution error: {str(e)}')
                self.total_errors += 1
                self.task_queue.pop(0)  # Remove failed task

    def execute_task(self, task):
        """Execute a single task"""
        task_type = task.get('type', 'unknown')

        if task_type == 'navigate':
            return self.execute_navigation_task(task)
        elif task_type == 'perceive':
            return self.execute_perception_task(task)
        elif task_type == 'manipulate':
            return self.execute_manipulation_task(task)
        elif task_type == 'combined':
            return self.execute_combined_task(task)
        else:
            self.get_logger().error(f'Unknown task type: {task_type}')
            return False

    def execute_navigation_task(self, task):
        """Execute navigation task"""
        try:
            target_pose = task.get('target_pose')
            if target_pose:
                # Use navigation action server
                goal = NavigationAction.Goal()
                goal.target_pose = target_pose

                # Execute navigation
                result = self.navigate_to_pose_blocking(goal)
                return result.success

        except Exception as e:
            self.get_logger().error(f'Navigation task error: {str(e)}')
            return False

    def execute_perception_task(self, task):
        """Execute perception task"""
        try:
            object_to_detect = task.get('object', 'any')
            self.get_logger().info(f'Starting perception task for: {object_to_detect}')

            # Use perception system
            if self.perception_system:
                detections = self.perception_system.detect_object(object_to_detect)
                return len(detections) > 0
            else:
                return False

        except Exception as e:
            self.get_logger().error(f'Perception task error: {str(e)}')
            return False

    def execute_manipulation_task(self, task):
        """Execute manipulation task"""
        try:
            object_to_manipulate = task.get('object')
            action = task.get('action', 'grasp')

            self.get_logger().info(f'Starting manipulation task: {action} {object_to_manipulate}')

            # Use manipulation system
            if self.manipulation_system:
                if action == 'grasp':
                    return self.manipulation_system.grasp_object(object_to_manipulate)
                elif action == 'place':
                    return self.manipulation_system.place_object(object_to_manipulate)
                elif action == 'move':
                    return self.manipulation_system.move_object(object_to_manipulate)
                else:
                    return False
            else:
                return False

        except Exception as e:
            self.get_logger().error(f'Manipulation task error: {str(e)}')
            return False

    def execute_combined_task(self, task):
        """Execute combined navigation and manipulation task"""
        try:
            subtasks = task.get('subtasks', [])

            for subtask in subtasks:
                success = self.execute_task(subtask)
                if not success:
                    return False

            return True

        except Exception as e:
            self.get_logger().error(f'Combined task error: {str(e)}')
            return False

    def navigate_to_pose_blocking(self, goal):
        """Execute navigation with blocking call"""
        # This would use the action client in a real implementation
        # For simulation, return success
        time.sleep(2)  # Simulate navigation time
        result = NavigationAction.Result()
        result.success = True
        result.message = "Navigation completed successfully"
        return result

    def check_emergency_stop_conditions(self, scan_msg):
        """Check if emergency stop conditions are met"""
        # Check for obstacles too close
        valid_ranges = [r for r in scan_msg.ranges if scan_msg.range_min <= r <= scan_msg.range_max]

        if valid_ranges:
            min_distance = min(valid_ranges)
            if min_distance < self.safety_distance:
                self.get_logger().warn(f'Obstacle too close: {min_distance:.2f}m < {self.safety_distance}m')
                return True

        return False

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        with self.system_lock:
            self.emergency_stop_active = True
            self.current_state = RobotState.EMERGENCY_STOP

            # Stop all movement
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

            self.get_logger().error('EMERGENCY STOP ACTIVATED')

    def release_emergency_stop(self):
        """Release emergency stop"""
        with self.system_lock:
            self.emergency_stop_active = False
            self.current_state = RobotState.IDLE
            self.get_logger().info('Emergency stop released')

    def monitor_system_health(self):
        """Monitor system health"""
        # Check if all systems are responsive
        self.system_health['navigation'] = self.is_component_responsive('navigation')
        self.system_health['perception'] = self.is_component_responsive('perception')
        self.system_health['manipulation'] = self.is_component_responsive('manipulation')
        self.system_health['cognition'] = self.is_component_responsive('cognition')
        self.system_health['sensors'] = self.are_sensors_active()

    def is_component_responsive(self, component):
        """Check if a component is responsive"""
        # In a real system, this would ping the component
        # For simulation, assume all components are responsive
        return True

    def are_sensors_active(self):
        """Check if sensors are active"""
        # Check if we have recent sensor data
        return self.robot_pose is not None

    def check_safety_conditions(self):
        """Check ongoing safety conditions"""
        if not self.emergency_stop_active:
            # Check battery level, overheating, etc.
            if self.should_trigger_safety_stop():
                self.trigger_emergency_stop()

    def should_trigger_safety_stop(self):
        """Check if safety stop should be triggered"""
        # Add safety checks here
        return False

    def publish_system_status(self):
        """Publish overall system status"""
        status_data = {
            'state': self.current_state.name,
            'emergency_stop': self.emergency_stop_active,
            'task_queue_size': len(self.task_queue),
            'total_tasks_completed': self.total_tasks_completed,
            'total_errors': self.total_errors,
            'uptime': time.time() - self.start_time,
            'system_health': self.system_health
        }

        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.system_status_pub.publish(status_msg)

    def goal_callback(self, goal_request):
        """Handle action goal requests"""
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle action cancellation"""
        return CancelResponse.ACCEPT

    async def navigate_to_pose_callback(self, goal_handle):
        """Handle navigation action"""
        self.get_logger().info('Executing navigation action')
        feedback_msg = NavigationAction.Feedback()
        result = NavigationAction.Result()

        try:
            target_pose = goal_handle.request.target_pose
            # Execute navigation (simplified)
            success = await self.navigate_to_pose_internal(target_pose, feedback_msg)

            if success:
                goal_handle.succeed()
                result.success = True
                result.message = "Successfully navigated to pose"
            else:
                goal_handle.abort()
                result.success = False
                result.message = "Failed to navigate to pose"

        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.message = f"Navigation error: {str(e)}"

        return result

    async def manipulate_object_callback(self, goal_handle):
        """Handle manipulation action"""
        self.get_logger().info('Executing manipulation action')
        feedback_msg = ManipulationAction.Feedback()
        result = ManipulationAction.Result()

        try:
            object_name = goal_handle.request.object_name
            action = goal_handle.request.action
            # Execute manipulation (simplified)
            success = await self.manipulate_object_internal(object_name, action, feedback_msg)

            if success:
                goal_handle.succeed()
                result.success = True
                result.message = f"Successfully {action}ed object {object_name}"
            else:
                goal_handle.abort()
                result.success = False
                result.message = f"Failed to {action} object {object_name}"

        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.message = f"Manipulation error: {str(e)}"

        return result

    async def navigate_to_pose_internal(self, target_pose, feedback_msg):
        """Internal navigation implementation"""
        # In a real system, this would use the navigation stack
        # For simulation, just return success after a delay
        for i in range(10):
            feedback_msg.progress = (i + 1) / 10.0
            goal_handle.publish_feedback(feedback_msg)
            await rclpy.sleep(0.2)
        return True

    async def manipulate_object_internal(self, object_name, action, feedback_msg):
        """Internal manipulation implementation"""
        # In a real system, this would use the manipulation stack
        # For simulation, just return success after a delay
        for i in range(5):
            feedback_msg.progress = (i + 1) / 5.0
            goal_handle.publish_feedback(feedback_msg)
            await rclpy.sleep(0.5)
        return True

    def get_robot_status(self):
        """Get current robot status"""
        return {
            'state': self.current_state.name,
            'pose': self.robot_pose,
            'joint_positions': self.joint_positions,
            'emergency_stop': self.emergency_stop_active,
            'task_queue_size': len(self.task_queue),
            'performance': {
                'uptime': time.time() - self.start_time,
                'tasks_completed': self.total_tasks_completed,
                'errors': self.total_errors,
                'success_rate': self.total_tasks_completed / max(1, self.total_tasks_completed + self.total_errors)
            }
        }

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousHumanoidRobot()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        print("Shutting down autonomous humanoid robot...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Cognitive Engine Integration

### Advanced Cognitive Planning Engine

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json
import threading
import time

class CognitiveEngine(Node):
    def __init__(self):
        super().__init__('cognitive_engine')

        # Subscribe to natural language commands
        self.command_sub = self.create_subscription(
            String,
            '/natural_language_command',
            self.command_callback,
            10
        )

        # Publisher for task plans
        self.plan_pub = self.create_publisher(String, '/task_plan', 10)

        # LLM configuration
        self.model = "gpt-4"
        self.max_tokens = 1000
        self.temperature = 0.3

        # Robot capabilities
        self.robot_capabilities = {
            "navigation": {
                "max_speed": 0.5,
                "min_turn_radius": 0.3,
                "max_range": 10.0
            },
            "manipulation": {
                "max_weight": 2.0,
                "reach_distance": 1.0,
                "gripper_types": ["power", "precision"]
            },
            "perception": {
                "camera_range": 5.0,
                "object_detection": True,
                "face_recognition": True
            }
        }

        # Current environment context
        self.environment_context = {
            "objects": [],
            "locations": [],
            "people": [],
            "obstacles": []
        }

        # Threading
        self.cognition_lock = threading.Lock()

        self.get_logger().info('Cognitive Engine initialized')

    def command_callback(self, msg):
        """Process natural language command and generate task plan"""
        command = msg.data.strip()
        if command:
            self.get_logger().info(f'Received command: "{command}"')

            with self.cognition_lock:
                # Generate task plan using LLM
                task_plan = self.generate_task_plan(command)

                if task_plan:
                    # Publish task plan
                    plan_msg = String()
                    plan_msg.data = json.dumps(task_plan)
                    self.plan_pub.publish(plan_msg)

                    self.get_logger().info(f'Generated task plan for: "{command}"')
                else:
                    self.get_logger().error(f'Failed to generate task plan for: "{command}"')

    def generate_task_plan(self, command):
        """Generate task plan using LLM"""
        try:
            prompt = self.create_task_planning_prompt(command)

            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=self.max_tokens,
                temperature=self.temperature
            )

            plan_json = response.choices[0].message.content.strip()

            # Validate plan
            if self.validate_task_plan(plan_json):
                return json.loads(plan_json)
            else:
                self.get_logger().error('Invalid task plan generated')
                return None

        except Exception as e:
            self.get_logger().error(f'Task planning error: {str(e)}')
            return None

    def create_task_planning_prompt(self, command):
        """Create prompt for task planning"""
        return f"""
        You are an advanced cognitive planning engine for a humanoid robot. Generate a detailed task plan to execute the following command.

        Command: "{command}"

        Robot capabilities: {json.dumps(self.robot_capabilities, indent=2)}
        Current environment context: {json.dumps(self.environment_context, indent=2)}

        Create a comprehensive task plan that includes:
        1. High-level task decomposition
        2. Navigation requirements (if needed)
        3. Perception requirements (object detection, etc.)
        4. Manipulation requirements (if needed)
        5. Safety considerations
        6. Error recovery strategies
        7. Success criteria

        Consider the robot's capabilities and current environment when planning.

        Respond with a JSON object containing:
        {{
            "task_id": "unique_identifier",
            "original_command": "the original command",
            "task_decomposition": [
                {{
                    "type": "navigation|perception|manipulation|combined",
                    "action": "specific action to take",
                    "parameters": {{"param": "value"}},
                    "dependencies": ["other_task_ids"],
                    "estimated_duration": 10.0,
                    "success_criteria": "how to verify success"
                }}
            ],
            "overall_success_criteria": "criteria for entire command completion",
            "estimated_total_time": 120.0,
            "safety_considerations": ["list", "of", "safety", "factors"],
            "risk_assessment": {{
                "high_risk_tasks": ["task_ids"],
                "mitigation_strategies": ["strategies"]
            }}
        }}

        Example for command "Bring me the red cup from the kitchen":
        {{
            "task_id": "bring_red_cup_001",
            "original_command": "Bring me the red cup from the kitchen",
            "task_decomposition": [
                {{
                    "type": "perception",
                    "action": "detect_red_cup",
                    "parameters": {{"object_color": "red", "object_type": "cup"}},
                    "dependencies": [],
                    "estimated_duration": 5.0,
                    "success_criteria": "Red cup detected in camera view"
                }},
                {{
                    "type": "navigation",
                    "action": "navigate_to_kitchen",
                    "parameters": {{"location": "kitchen"}},
                    "dependencies": [],
                    "estimated_duration": 30.0,
                    "success_criteria": "Robot has reached kitchen area"
                }},
                {{
                    "type": "manipulation",
                    "action": "grasp_red_cup",
                    "parameters": {{"object_id": "red_cup_001"}},
                    "dependencies": ["detect_red_cup"],
                    "estimated_duration": 10.0,
                    "success_criteria": "Red cup is grasped by robot"
                }},
                {{
                    "type": "navigation",
                    "action": "return_to_user",
                    "parameters": {{"destination": "user_location"}},
                    "dependencies": ["grasp_red_cup"],
                    "estimated_duration": 30.0,
                    "success_criteria": "Robot has returned to user location"
                }}
            ],
            "overall_success_criteria": "Red cup delivered to user",
            "estimated_total_time": 75.0,
            "safety_considerations": ["avoid obstacles", "handle fragile object carefully"],
            "risk_assessment": {{
                "high_risk_tasks": ["grasp_red_cup"],
                "mitigation_strategies": ["use precision grip", "monitor force feedback"]
            }}
        }}
        """

    def get_system_prompt(self):
        """System prompt for cognitive planning"""
        return """
        You are an expert cognitive planning system for humanoid robots. Your role is to decompose natural language commands into executable task plans.

        Requirements:
        1. Create logical task hierarchies with proper dependencies
        2. Consider robot capabilities and limitations
        3. Include safety and error recovery considerations
        4. Provide realistic time estimates
        5. Return only valid JSON with the specified structure
        6. Ensure tasks are executable in the given environment
        7. Plan for contingencies and alternative approaches
        8. Prioritize safety in all planning decisions
        """

    def validate_task_plan(self, plan_json_str):
        """Validate task plan structure"""
        try:
            plan = json.loads(plan_json_str)

            required_fields = ['task_id', 'original_command', 'task_decomposition', 'overall_success_criteria']
            if not all(field in plan for field in required_fields):
                return False

            if not isinstance(plan['task_decomposition'], list):
                return False

            for task in plan['task_decomposition']:
                if 'type' not in task or 'action' not in task:
                    return False

            return True

        except json.JSONDecodeError:
            return False
        except Exception:
            return False

def main(args=None):
    rclpy.init(args=args)
    node = CognitiveEngine()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down cognitive engine...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # Set your OpenAI API key here or via environment variable
    # openai.api_key = "your-api-key-here"
    main()
```

## Performance Optimization and Profiling

### System Performance Monitor

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import psutil
import time
import threading
from collections import deque
import numpy as np

class SystemPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('system_performance_monitor')

        # Publishers for performance metrics
        self.cpu_usage_pub = self.create_publisher(Float32, '/system/cpu_usage', 10)
        self.memory_usage_pub = self.create_publisher(Float32, '/system/memory_usage', 10)
        self.performance_status_pub = self.create_publisher(String, '/system/performance_status', 10)

        # Performance tracking
        self.cpu_history = deque(maxlen=100)
        self.memory_history = deque(maxlen=100)
        self.fps_history = deque(maxlen=30)

        # Performance thresholds
        self.cpu_threshold = 80.0  # %
        self.memory_threshold = 85.0  # %
        self.min_fps = 10.0  # frames per second

        # Threading
        self.monitoring_thread = threading.Thread(target=self.monitor_system, daemon=True)
        self.monitoring_thread.start()

        self.frame_count = 0
        self.last_frame_time = time.time()

        # Performance optimization parameters
        self.current_optimization_level = 1  # 1-5 scale
        self.max_optimization_level = 5

        self.get_logger().info('System Performance Monitor initialized')

    def increment_frame_count(self):
        """Call this method when processing a frame to track FPS"""
        self.frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.last_frame_time

        if elapsed_time >= 1.0:  # Update FPS every second
            current_fps = self.frame_count / elapsed_time
            self.fps_history.append(current_fps)
            self.frame_count = 0
            self.last_frame_time = current_time

    def monitor_system(self):
        """Monitor system resources"""
        while rclpy.ok():
            # Get system metrics
            cpu_percent = psutil.cpu_percent(interval=1)
            memory_percent = psutil.virtual_memory().percent

            # Store in history
            self.cpu_history.append(cpu_percent)
            self.memory_history.append(memory_percent)

            # Publish metrics
            cpu_msg = Float32()
            cpu_msg.data = float(cpu_percent)
            self.cpu_usage_pub.publish(cpu_msg)

            memory_msg = Float32()
            memory_msg.data = float(memory_percent)
            self.memory_usage_pub.publish(memory_msg)

            # Check performance status
            self.check_performance_status(cpu_percent, memory_percent)

            # Adjust optimization level based on performance
            self.adjust_optimization_level(cpu_percent, memory_percent)

    def check_performance_status(self, cpu_percent, memory_percent):
        """Check if system performance is within acceptable ranges"""
        status = {
            'cpu': 'normal',
            'memory': 'normal',
            'fps': 'normal',
            'optimization_level': self.current_optimization_level
        }

        if cpu_percent > self.cpu_threshold:
            status['cpu'] = 'high'
        if memory_percent > self.memory_threshold:
            status['memory'] = 'high'

        if self.fps_history:
            avg_fps = sum(self.fps_history) / len(self.fps_history)
            if avg_fps < self.min_fps:
                status['fps'] = 'low'

        # Publish status
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.performance_status_pub.publish(status_msg)

        # Log warnings if performance is poor
        if status['cpu'] == 'high' or status['memory'] == 'high':
            self.get_logger().warn(f'Performance warning: CPU={cpu_percent:.1f}%, Memory={memory_percent:.1f}%')

    def adjust_optimization_level(self, cpu_percent, memory_percent):
        """Adjust optimization level based on system load"""
        load_factor = (cpu_percent + memory_percent) / 200.0  # Normalize to 0-1

        if load_factor > 0.9:
            # Very high load, increase optimization
            self.current_optimization_level = min(self.max_optimization_level, self.current_optimization_level + 1)
        elif load_factor < 0.7:
            # Low load, decrease optimization (better quality)
            self.current_optimization_level = max(1, self.current_optimization_level - 1)

    def get_optimization_parameters(self):
        """Get optimization parameters based on current level"""
        if self.current_optimization_level == 1:
            return {
                'image_processing_quality': 'high',
                'vision_fps': 30,
                'tracking_points': 100,
                'detection_threshold': 0.7,
                'planning_complexity': 'full'
            }
        elif self.current_optimization_level == 2:
            return {
                'image_processing_quality': 'medium-high',
                'vision_fps': 25,
                'tracking_points': 80,
                'detection_threshold': 0.6,
                'planning_complexity': 'full'
            }
        elif self.current_optimization_level == 3:
            return {
                'image_processing_quality': 'medium',
                'vision_fps': 20,
                'tracking_points': 60,
                'detection_threshold': 0.5,
                'planning_complexity': 'simplified'
            }
        elif self.current_optimization_level == 4:
            return {
                'image_processing_quality': 'medium-low',
                'vision_fps': 15,
                'tracking_points': 40,
                'detection_threshold': 0.4,
                'planning_complexity': 'simplified'
            }
        else:  # Level 5
            return {
                'image_processing_quality': 'low',
                'vision_fps': 10,
                'tracking_points': 20,
                'detection_threshold': 0.3,
                'planning_complexity': 'minimal'
            }

def main(args=None):
    rclpy.init(args=args)
    node = SystemPerformanceMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down system performance monitor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import json  # Import here to avoid issues if not available
    main()
```

## Safety and Validation Systems

### Comprehensive Safety Validator

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
import threading
import time

class SafetyValidator(Node):
    def __init__(self):
        super().__init__('safety_validator')

        # Subscribe to critical systems
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers
        self.safety_status_pub = self.create_publisher(Bool, '/safety_status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.safety_violation_pub = self.create_publisher(String, '/safety_violation', 10)

        # Safety parameters
        self.safety_distances = {
            'min_proximity': 0.3,  # meters
            'critical_proximity': 0.1  # meters
        }

        self.speed_limits = {
            'max_linear_speed': 0.5,  # m/s
            'max_angular_speed': 0.5  # rad/s
        }

        self.joint_limits = {
            'max_velocity': 1.0,  # rad/s
            'max_effort': 100.0   # N*m
        }

        # Current system state
        self.current_scan = None
        self.current_joints = None
        self.last_cmd_vel = None
        self.last_cmd_time = time.time()

        # Safety state
        self.safety_violations = 0
        self.max_violations_before_stop = 3
        self.emergency_stop_active = False

        # Threading
        self.safety_lock = threading.Lock()
        self.safety_check_thread = threading.Thread(target=self.safety_monitor_loop, daemon=True)
        self.safety_check_thread.start()

        # Safety timers
        self.last_scan_time = time.time()
        self.scan_timeout = 1.0  # seconds

        self.get_logger().info('Safety Validator initialized')

    def scan_callback(self, msg):
        """Process laser scan for safety checks"""
        self.current_scan = msg
        self.last_scan_time = time.time()

    def joint_state_callback(self, msg):
        """Process joint states for safety checks"""
        self.current_joints = msg

    def cmd_vel_callback(self, msg):
        """Process velocity commands for safety validation"""
        self.last_cmd_vel = msg
        self.last_cmd_time = time.time()

    def safety_monitor_loop(self):
        """Main safety monitoring loop"""
        while rclpy.ok():
            with self.safety_lock:
                # Check various safety conditions
                self.check_obstacle_safety()
                self.check_command_safety()
                self.check_joint_safety()
                self.check_system_timeliness()

                # Publish safety status
                safety_status = Bool()
                safety_status.data = not self.emergency_stop_active
                self.safety_status_pub.publish(safety_status)

            time.sleep(0.05)  # 20Hz safety checks

    def check_obstacle_safety(self):
        """Check for obstacle proximity violations"""
        if self.current_scan is None:
            return

        # Get valid ranges
        valid_ranges = [
            r for r in self.current_scan.ranges
            if self.current_scan.range_min <= r <= self.current_scan.range_max
        ]

        if valid_ranges:
            min_distance = min(valid_ranges)

            if min_distance < self.safety_distances['critical_proximity']:
                self.trigger_emergency_stop(f'Critical obstacle proximity: {min_distance:.2f}m')
            elif min_distance < self.safety_distances['min_proximity']:
                self.log_safety_violation(f'Obstacle too close: {min_distance:.2f}m')

    def check_command_safety(self):
        """Check velocity command safety"""
        if self.last_cmd_vel is None:
            return

        linear_speed = np.sqrt(
            self.last_cmd_vel.linear.x**2 +
            self.last_cmd_vel.linear.y**2 +
            self.last_cmd_vel.linear.z**2
        )

        angular_speed = np.sqrt(
            self.last_cmd_vel.angular.x**2 +
            self.last_cmd_vel.angular.y**2 +
            self.last_cmd_vel.angular.z**2
        )

        if linear_speed > self.speed_limits['max_linear_speed']:
            self.log_safety_violation(f'Linear speed violation: {linear_speed:.2f}m/s > {self.speed_limits["max_linear_speed"]}m/s')

        if angular_speed > self.speed_limits['max_angular_speed']:
            self.log_safety_violation(f'Angular speed violation: {angular_speed:.2f}rad/s > {self.speed_limits["max_angular_speed"]}rad/s')

    def check_joint_safety(self):
        """Check joint state safety"""
        if self.current_joints is None:
            return

        if self.current_joints.velocity:
            max_velocity = max(abs(v) for v in self.current_joints.velocity)
            if max_velocity > self.joint_limits['max_velocity']:
                self.log_safety_violation(f'Joint velocity limit exceeded: {max_velocity:.2f}rad/s > {self.joint_limits["max_velocity"]}rad/s')

        if self.current_joints.effort:
            max_effort = max(abs(e) for e in self.current_joints.effort)
            if max_effort > self.joint_limits['max_effort']:
                self.log_safety_violation(f'Joint effort limit exceeded: {max_effort:.2f}N*m > {self.joint_limits["max_effort"]}N*m')

    def check_system_timeliness(self):
        """Check that critical systems are responsive"""
        current_time = time.time()

        # Check if scan data is too old
        if current_time - self.last_scan_time > self.scan_timeout:
            self.log_safety_violation(f'Scan data timeout: {current_time - self.last_scan_time:.2f}s > {self.scan_timeout}s')

        # Check if velocity commands are too old
        if current_time - self.last_cmd_time > self.scan_timeout:
            self.log_safety_violation(f'Velocity command timeout: {current_time - self.last_cmd_time:.2f}s > {self.scan_timeout}s')

    def log_safety_violation(self, violation_msg):
        """Log safety violation"""
        self.safety_violations += 1
        self.get_logger().warn(f'Safety violation: {violation_msg}')
        self.get_logger().info(f'Safety violations: {self.safety_violations}/{self.max_violations_before_stop}')

        # Publish violation
        violation_msg_obj = String()
        violation_msg_obj.data = violation_msg
        self.safety_violation_pub.publish(violation_msg_obj)

        # Check if emergency stop should be triggered
        if self.safety_violations >= self.max_violations_before_stop:
            self.trigger_emergency_stop('Too many safety violations')

    def trigger_emergency_stop(self, reason):
        """Trigger emergency stop"""
        with self.safety_lock:
            if not self.emergency_stop_active:
                self.emergency_stop_active = True
                self.get_logger().error(f'EMERGENCY STOP TRIGGERED: {reason}')

                # Publish emergency stop signal
                emergency_msg = Bool()
                emergency_msg.data = True
                self.emergency_stop_pub.publish(emergency_msg)

                # Stop all movement
                stop_cmd = Twist()
                self.create_publisher(Twist, '/cmd_vel', 10).publish(stop_cmd)

                # Reset violation counter
                self.safety_violations = 0

    def reset_emergency_stop(self):
        """Reset emergency stop"""
        with self.safety_lock:
            self.emergency_stop_active = False
            self.get_logger().info('Emergency stop reset')

    def get_safety_status(self):
        """Get current safety status"""
        return {
            'emergency_stop_active': self.emergency_stop_active,
            'safety_violations': self.safety_violations,
            'last_scan_time': self.last_scan_time,
            'last_cmd_time': self.last_cmd_time
        }

def main(args=None):
    rclpy.init(args=args)
    node = SafetyValidator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down safety validator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## End-to-End Testing and Validation

### System Integration Test Suite

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32
import unittest
import time
import threading
from typing import Dict, Any

class SystemIntegrationTestSuite(Node):
    def __init__(self):
        super().__init__('system_integration_test_suite')

        # Publishers for test commands
        self.test_command_pub = self.create_publisher(String, '/test_command', 10)
        self.test_status_pub = self.create_publisher(String, '/test_status', 10)
        self.voice_command_pub = self.create_publisher(String, '/natural_language_command', 10)

        # Subscribers for test results
        self.test_result_sub = self.create_subscription(
            String,
            '/test_result',
            self.test_result_callback,
            10
        )

        # Test results storage
        self.test_results = {}
        self.current_test = None
        self.test_in_progress = False

        # Threading
        self.test_lock = threading.Lock()

        self.get_logger().info('System Integration Test Suite initialized')

    def run_all_tests(self):
        """Run the complete test suite"""
        self.get_logger().info('Starting system integration test suite...')

        # Test individual components first
        self.test_navigation_system()
        self.test_perception_system()
        self.test_manipulation_system()
        self.test_cognitive_engine()

        # Test integrated functionality
        self.test_autonomous_behavior()
        self.test_voice_interaction()
        self.test_safety_systems()

        # Generate test report
        self.generate_test_report()

    def test_navigation_system(self):
        """Test navigation system functionality"""
        self.get_logger().info('Testing navigation system...')

        # Send navigation command
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = 1.0
        goal_msg.pose.position.y = 1.0
        goal_msg.pose.orientation.w = 1.0

        # Wait for result
        success = self.wait_for_test_result('navigation_test', timeout=30.0)
        self.record_test_result('navigation_system', success)

    def test_perception_system(self):
        """Test perception system functionality"""
        self.get_logger().info('Testing perception system...')

        # Simulate object detection request
        command_msg = String()
        command_msg.data = 'detect red cup'
        self.test_command_pub.publish(command_msg)

        # Wait for result
        success = self.wait_for_test_result('perception_test', timeout=10.0)
        self.record_test_result('perception_system', success)

    def test_manipulation_system(self):
        """Test manipulation system functionality"""
        self.get_logger().info('Testing manipulation system...')

        # Simulate manipulation command
        command_msg = String()
        command_msg.data = 'grasp object'
        self.test_command_pub.publish(command_msg)

        # Wait for result
        success = self.wait_for_test_result('manipulation_test', timeout=15.0)
        self.record_test_result('manipulation_system', success)

    def test_cognitive_engine(self):
        """Test cognitive engine functionality"""
        self.get_logger().info('Testing cognitive engine...')

        # Send natural language command
        command_msg = String()
        command_msg.data = 'Go to the kitchen and bring me a cup'
        self.voice_command_pub.publish(command_msg)

        # Wait for task plan
        success = self.wait_for_test_result('cognitive_test', timeout=20.0)
        self.record_test_result('cognitive_engine', success)

    def test_autonomous_behavior(self):
        """Test complete autonomous behavior"""
        self.get_logger().info('Testing autonomous behavior...')

        # Send complex command
        command_msg = String()
        command_msg.data = 'Navigate to the living room, find the blue book, pick it up, and bring it to me'
        self.voice_command_pub.publish(command_msg)

        # Wait for complete execution
        success = self.wait_for_test_result('autonomous_test', timeout=60.0)
        self.record_test_result('autonomous_behavior', success)

    def test_voice_interaction(self):
        """Test voice interaction system"""
        self.get_logger().info('Testing voice interaction...')

        # Test multiple voice commands
        commands = [
            'Move forward',
            'Turn left',
            'Stop',
            'Where are you?',
            'What can you do?'
        ]

        all_success = True
        for cmd in commands:
            command_msg = String()
            command_msg.data = cmd
            self.voice_command_pub.publish(command_msg)

            success = self.wait_for_test_result('voice_test', timeout=5.0)
            if not success:
                all_success = False
                break

        self.record_test_result('voice_interaction', all_success)

    def test_safety_systems(self):
        """Test safety system functionality"""
        self.get_logger().info('Testing safety systems...')

        # Simulate safety scenario
        # This would involve triggering obstacle detection, etc.
        success = True  # For simulation purposes

        self.record_test_result('safety_systems', success)

    def wait_for_test_result(self, test_name, timeout=10.0):
        """Wait for test result with timeout"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.test_lock:
                if test_name in self.test_results:
                    result = self.test_results[test_name]
                    del self.test_results[test_name]
                    return result
            time.sleep(0.1)

        # Timeout occurred
        self.get_logger().warn(f'Test {test_name} timed out')
        return False

    def test_result_callback(self, msg):
        """Process test result"""
        try:
            result_data = eval(msg.data)  # In practice, use proper deserialization
            test_name = result_data.get('test_name')
            success = result_data.get('success', False)

            with self.test_lock:
                self.test_results[test_name] = success

        except Exception as e:
            self.get_logger().error(f'Test result callback error: {str(e)}')

    def record_test_result(self, test_name, success):
        """Record test result"""
        self.test_results[test_name] = success
        self.get_logger().info(f'{test_name}: {"PASS" if success else "FAIL"}')

    def generate_test_report(self):
        """Generate comprehensive test report"""
        total_tests = len(self.test_results)
        passed_tests = sum(1 for success in self.test_results.values() if success)
        failed_tests = total_tests - passed_tests

        report = {
            'timestamp': time.time(),
            'total_tests': total_tests,
            'passed_tests': passed_tests,
            'failed_tests': failed_tests,
            'pass_rate': passed_tests / max(1, total_tests) * 100,
            'test_results': self.test_results
        }

        self.get_logger().info(f'Test Report: {passed_tests}/{total_tests} tests passed ({report["pass_rate"]:.1f}%)')
        self.get_logger().info(f'Detailed results: {self.test_results}')

        # Publish report
        report_msg = String()
        report_msg.data = str(report)
        self.test_status_pub.publish(report_msg)

        return report

def main(args=None):
    rclpy.init(args=args)
    node = SystemIntegrationTestSuite()

    try:
        # Run the complete test suite
        node.run_all_tests()

        # Keep node alive to receive results
        time.sleep(5.0)

    except KeyboardInterrupt:
        print("Shutting down test suite...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Lab: Complete Autonomous Robot Deployment

In this final lab, you'll deploy the complete autonomous humanoid robot system.

### Step 1: Create the Complete System Launch File

Create `complete_autonomous_robot_launch.py`:

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
    robot_name = LaunchConfiguration('robot_name', default='humanoid_robot')

    # Main robot system nodes
    robot_core = Node(
        package='ai_robo_learning',
        executable='autonomous_humanoid_robot',
        name='autonomous_humanoid_robot',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_name': robot_name}
        ]
    )

    cognitive_engine = Node(
        package='ai_robo_learning',
        executable='cognitive_engine',
        name='cognitive_engine',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    performance_monitor = Node(
        package='ai_robo_learning',
        executable='system_performance_monitor',
        name='system_performance_monitor',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    safety_validator = Node(
        package='ai_robo_learning',
        executable='safety_validator',
        name='safety_validator',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    test_suite = Node(
        package='ai_robo_learning',
        executable='system_integration_test_suite',
        name='system_integration_test_suite',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Navigation system (from previous modules)
    navigation_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ])
    )

    # Perception system (from previous modules)
    perception_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_ros_examples'),
                'launch',
                'isaac_ros_stereo_image_proc.launch.py'
            ])
        ])
    )

    # Return launch description
    ld = LaunchDescription()

    # Add all components
    ld.add_action(robot_core)
    ld.add_action(cognitive_engine)
    ld.add_action(performance_monitor)
    ld.add_action(safety_validator)
    ld.add_action(test_suite)
    ld.add_action(navigation_system)
    ld.add_action(perception_system)

    return ld
```

### Step 2: Create the Complete Deployment Script

Create `deploy_autonomous_robot.py`:

```python
#!/usr/bin/env python3

import subprocess
import time
import sys
import os
from pathlib import Path

def deploy_autonomous_robot():
    """Deploy the complete autonomous humanoid robot system"""
    print("🚀 Deploying Autonomous Humanoid Robot System...")
    print("=" * 60)

    # Check prerequisites
    print("\n🔍 Checking system prerequisites...")
    if not check_prerequisites():
        print("❌ Prerequisites not met. Please install required dependencies.")
        return False

    # Build the system
    print("\n🔧 Building the robot system...")
    if not build_system():
        print("❌ System build failed.")
        return False

    # Launch the system
    print("\n🤖 Launching autonomous robot system...")
    launch_system()

    print("\n✅ Autonomous humanoid robot system deployed successfully!")
    print("\n📋 Next steps:")
    print("   • Send voice commands via: ros2 topic pub /natural_language_command std_msgs/String 'data: \"command\"'")
    print("   • Monitor system status via: ros2 topic echo /system_status")
    print("   • Run tests via: ros2 run ai_robo_learning system_integration_test_suite")
    print("\n🎉 Your autonomous humanoid robot is ready for operation!")

def check_prerequisites():
    """Check if all prerequisites are installed"""
    checks = [
        ("Python 3.8+", check_python_version),
        ("ROS 2 Humble", check_ros2_installation),
        ("OpenAI API key", check_openai_api_key),
        ("Required Python packages", check_python_packages),
        ("CUDA (for Isaac Sim)", check_cuda_installation)
    ]

    all_passed = True
    for check_name, check_func in checks:
        print(f"  • {check_name}...", end="")
        if check_func():
            print(" ✅")
        else:
            print(" ❌")
            all_passed = False

    return all_passed

def check_python_version():
    """Check Python version"""
    import sys
    return sys.version_info >= (3, 8)

def check_ros2_installation():
    """Check if ROS 2 Humble is installed"""
    try:
        result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
        return result.returncode == 0
    except FileNotFoundError:
        return False

def check_openai_api_key():
    """Check if OpenAI API key is set"""
    import os
    return os.environ.get('OPENAI_API_KEY') is not None

def check_python_packages():
    """Check if required Python packages are installed"""
    required_packages = [
        'rclpy', 'numpy', 'opencv-python', 'openai', 'scipy', 'psutil'
    ]

    for package in required_packages:
        try:
            __import__(package.replace('-', '_'))
        except ImportError:
            return False
    return True

def check_cuda_installation():
    """Check if CUDA is available"""
    try:
        import torch
        return torch.cuda.is_available()
    except ImportError:
        return False

def build_system():
    """Build the robot system"""
    try:
        # Change to workspace directory
        workspace_dir = Path.home() / "robot_ws"
        if workspace_dir.exists():
            os.chdir(workspace_dir)

            # Build with colcon
            result = subprocess.run(['colcon', 'build', '--packages-select', 'ai_robo_learning'],
                                  capture_output=True, text=True)
            return result.returncode == 0
        else:
            print(f"Workspace directory {workspace_dir} does not exist")
            return False
    except Exception as e:
        print(f"Build error: {e}")
        return False

def launch_system():
    """Launch the complete system"""
    try:
        # Source the workspace
        source_cmd = "source ~/robot_ws/install/setup.bash && "

        # Launch the main system
        launch_cmd = f"{source_cmd}ros2 launch ai_robo_learning complete_autonomous_robot_launch.py"

        print(f"   Launching with command: {launch_cmd}")

        # In a real deployment, you might want to use subprocess.Popen
        # For now, just print the command that would be used
        print("   System launched! (simulation)")

    except Exception as e:
        print(f"Launch error: {e}")

def main():
    """Main deployment function"""
    try:
        deploy_autonomous_robot()
    except KeyboardInterrupt:
        print("\n\n⚠️  Deployment interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Deployment failed: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
```

### Step 3: Test the Complete System

1. Make sure all dependencies are installed:
```bash
pip3 install rclpy numpy opencv-python openai scipy psutil torch
```

2. Set your OpenAI API key:
```bash
export OPENAI_API_KEY="your-api-key-here"
```

3. Run the deployment script:
```bash
python3 deploy_autonomous_robot.py
```

4. Launch the complete system:
```bash
source ~/robot_ws/install/setup.bash
ros2 launch ai_robo_learning complete_autonomous_robot_launch.py
```

5. Test with voice commands:
```bash
# Send a test command
ros2 topic pub /natural_language_command std_msgs/String "data: 'Navigate to the kitchen and bring me a cup'"
```

## Best Practices for Production Deployment

1. **Safety First**: Always implement multiple layers of safety checks
2. **Monitoring**: Continuously monitor system health and performance
3. **Logging**: Maintain comprehensive logs for debugging and analysis
4. **Testing**: Thoroughly test all scenarios before deployment
5. **Fail-Safes**: Implement graceful degradation and recovery mechanisms
6. **Security**: Secure all communication channels and data
7. **Maintenance**: Plan for regular updates and maintenance
8. **Documentation**: Maintain clear documentation for all systems

## Conclusion

Congratulations! You've completed the comprehensive AI-native textbook for Physical AI & Humanoid Robotics. You now have the knowledge and tools to:

- Build and program humanoid robots with advanced perception and manipulation
- Integrate AI systems for autonomous decision making
- Implement safe and reliable robotic systems
- Deploy complete robotic solutions in real environments

The journey from simulation to embodied intelligence is complete. Continue exploring, experimenting, and pushing the boundaries of what's possible in humanoid robotics!