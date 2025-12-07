---
sidebar_position: 5
title: 'rclpy and Python Integration'
---

# rclpy and Python Integration

This chapter focuses on integrating Python-based AI agents with ROS 2 systems using rclpy, the Python client library for ROS 2. You'll learn how to bridge AI algorithms with robotic control systems.

## What You'll Learn

In this chapter, you'll explore:
- Advanced rclpy concepts and patterns
- Integrating Python AI agents with ROS 2
- Creating custom message and service types
- Parameter management and configuration
- Advanced node composition and design patterns

## Prerequisites

- Completion of Module 1, Chapters 1-4
- Python programming skills
- Understanding of AI/ML concepts
- ROS 2 Humble installed on your system

## Advanced rclpy Concepts

### Node Composition

You can run multiple nodes in a single process using node composition:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1

class SubscriberNode(Node):

    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    publisher_node = PublisherNode()
    subscriber_node = SubscriberNode()

    executor = MultiThreadedExecutor()
    executor.add_node(publisher_node)
    executor.add_node(subscriber_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Lifecycle Nodes

Lifecycle nodes provide better resource management and coordination:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor

class LifecycleTalker(LifecycleNode):

    def __init__(self):
        super().__init__('lifecycle_talker')
        self.publisher = None
        self.timer = None
        self.i = 0

    def on_configure(self, state):
        self.get_logger().info(f'Configuring {self.get_name()}')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info(f'Activating {self.get_name()}')
        self.timer.reset()
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info(f'Deactivating {self.get_name()}')
        self.timer.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info(f'Cleaning up {self.get_name()}')
        self.timer.destroy()
        self.publisher.destroy()
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        msg = String()
        msg.data = f'Lifecycle message {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

## Integrating AI Agents with ROS 2

### AI Agent Node Example

Here's an example of integrating an AI agent that processes sensor data:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import random

class AIAgentNode(Node):

    def __init__(self):
        super().__init__('ai_agent_node')

        # Subscribe to laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publish velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for AI decision making
        self.timer = self.create_timer(0.1, self.ai_decision_callback)

        self.laser_data = None
        self.get_logger().info('AI Agent Node initialized')

    def laser_callback(self, msg):
        """Process incoming laser scan data"""
        self.laser_data = np.array(msg.ranges)
        # Replace invalid readings with max range
        self.laser_data[np.isnan(self.laser_data)] = msg.range_max
        self.laser_data[np.isinf(self.laser_data)] = msg.range_max

    def ai_decision_callback(self):
        """Make decisions based on sensor data"""
        if self.laser_data is None:
            return

        # Simple obstacle avoidance algorithm
        # Check if there's an obstacle in front
        front_range = self.laser_data[len(self.laser_data)//2 - 10:len(self.laser_data)//2 + 10]
        min_front_distance = np.min(front_range)

        cmd_vel = Twist()

        if min_front_distance < 1.0:  # Obstacle within 1 meter
            # Turn right
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = -0.5
            self.get_logger().info('Obstacle detected! Turning right')
        else:
            # Move forward
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0.0
            self.get_logger().info('Clear path, moving forward')

        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgentNode()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Custom Message Types

### Creating Custom Messages

To create custom messages, first create a `msg` directory in your ROS 2 package and define your message:

**HumanoidJointState.msg**:
```
# Custom message for humanoid robot joint states
string robot_name
float64[] positions
float64[] velocities
float64[] efforts
string[] joint_names
time header
```

### Using Custom Messages

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# Assuming you have a custom message package called 'humanoid_msgs'
# from humanoid_msgs.msg import HumanoidJointState

class HumanoidController(Node):

    def __init__(self):
        super().__init__('humanoid_controller')
        # self.publisher = self.create_publisher(
        #     HumanoidJointState,
        #     'humanoid_joint_states',
        #     10
        # )
        self.get_logger().info('Humanoid Controller initialized')

    def send_joint_commands(self, joint_positions):
        """Send joint position commands to the humanoid"""
        # msg = HumanoidJointState()
        # msg.robot_name = 'humanoid_robot'
        # msg.positions = joint_positions
        # msg.header.stamp = self.get_clock().now().to_msg()
        # self.publisher.publish(msg)
        pass

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Parameter Management

### Using Parameters in Nodes

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from std_msgs.msg import String

class ParameterizedNode(Node):

    def __init__(self):
        super().__init__('parameterized_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('safety_distance', 0.5)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value

        # Create publisher
        self.publisher = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f'Node initialized with parameters: '
            f'robot_name={self.robot_name}, '
            f'max_speed={self.max_speed}, '
            f'safety_distance={self.safety_distance}'
        )

    def publish_status(self):
        msg = String()
        msg.data = f'{self.robot_name} status: OK, speed={self.max_speed}'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    param_node = ParameterizedNode()

    try:
        rclpy.spin(param_node)
    except KeyboardInterrupt:
        pass
    finally:
        param_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-on Lab: AI-Driven Humanoid Controller

In this lab, you'll create an AI agent that controls a humanoid robot based on sensor inputs.

### Step 1: Create the AI Controller

Create `ai_humanoid_controller.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import time

class AIHumanoidController(Node):

    def __init__(self):
        super().__init__('ai_humanoid_controller')

        # Subscribe to joint states
        self.joint_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        # Subscribe to camera data (simulated)
        self.camera_subscriber = self.create_subscription(
            String,
            'camera_data',
            self.camera_callback,
            10)

        # Publish commands to robot
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for AI decision making
        self.timer = self.create_timer(0.2, self.ai_decision_callback)

        # Robot state
        self.joint_positions = {}
        self.camera_data = None
        self.ai_state = 'idle'  # idle, walking, avoiding_obstacle, etc.

        self.get_logger().info('AI Humanoid Controller initialized')

    def joint_state_callback(self, msg):
        """Process joint state messages"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def camera_callback(self, msg):
        """Process camera data"""
        self.camera_data = msg.data

    def ai_decision_callback(self):
        """Main AI decision making function"""
        # Simple state machine for humanoid behavior
        if self.ai_state == 'idle':
            self.get_logger().info('AI: In idle state')
            # Check if we have a command to start walking
            if self.camera_data and 'walk' in self.camera_data.lower():
                self.ai_state = 'walking'
                self.start_walking()
        elif self.ai_state == 'walking':
            # Check for obstacles or commands to stop
            if self.camera_data and 'stop' in self.camera_data.lower():
                self.ai_state = 'idle'
                self.stop_robot()
            else:
                # Continue walking
                self.walk_forward()

    def start_walking(self):
        """Start the walking behavior"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.3  # Move forward slowly
        cmd_vel.angular.z = 0.0
        self.cmd_publisher.publish(cmd_vel)
        self.get_logger().info('AI: Starting to walk')

    def walk_forward(self):
        """Continue walking forward"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.3
        cmd_vel.angular.z = 0.0
        self.cmd_publisher.publish(cmd_vel)

    def stop_robot(self):
        """Stop the robot"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_publisher.publish(cmd_vel)
        self.get_logger().info('AI: Stopping robot')

def main(args=None):
    rclpy.init(args=args)
    ai_controller = AIHumanoidController()

    try:
        rclpy.spin(ai_controller)
    except KeyboardInterrupt:
        pass
    finally:
        ai_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Create a Simple Test Node

Create `test_ai_input.py` to simulate camera input:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TestAIInput(Node):

    def __init__(self):
        super().__init__('test_ai_input')
        self.publisher = self.create_publisher(String, 'camera_data', 10)

        # Send commands at intervals
        self.commands = ['walk forward', 'stop', 'walk forward', 'stop']
        self.command_index = 0

        self.timer = self.create_timer(5.0, self.send_command)
        self.get_logger().info('Test AI Input node started')

    def send_command(self):
        if self.command_index < len(self.commands):
            msg = String()
            msg.data = self.commands[self.command_index]
            self.publisher.publish(msg)
            self.get_logger().info(f'Sent command: {msg.data}')
            self.command_index += 1

def main(args=None):
    rclpy.init(args=args)
    test_node = TestAIInput()

    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Run the AI Controller

1. Terminal 1 - Start the AI controller:
```bash
source /opt/ros/humble/setup.bash
python3 ai_humanoid_controller.py
```

2. Terminal 2 - Send test commands:
```bash
source /opt/ros/humble/setup.bash
python3 test_ai_input.py
```

## Advanced Patterns

### Async/Await Pattern

```python
import asyncio
from rclpy.node import Node
from rclpy.qos import QoSProfile

class AsyncNode(Node):

    def __init__(self):
        super().__init__('async_node')
        self.qos_profile = QoSProfile(depth=10)

    async def async_publisher(self):
        publisher = self.create_publisher(String, 'async_topic', self.qos_profile)
        for i in range(10):
            msg = String()
            msg.data = f'Async message {i}'
            publisher.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
            await asyncio.sleep(1.0)
```

### Threading Pattern

```python
import threading
from rclpy.node import Node

class ThreadingNode(Node):

    def __init__(self):
        super().__init__('threading_node')
        self.lock = threading.Lock()
        self.shared_data = {}

        # Start background thread
        self.bg_thread = threading.Thread(target=self.background_task)
        self.bg_thread.daemon = True
        self.bg_thread.start()

    def background_task(self):
        """Background task that updates shared data"""
        import time
        counter = 0
        while rclpy.ok():
            with self.lock:
                self.shared_data['counter'] = counter
                counter += 1
            time.sleep(0.1)
```

## Best Practices

1. **AI Integration**: Keep AI processing separate from ROS message handling to avoid blocking
2. **Threading**: Use threading carefully with ROS nodes to avoid race conditions
3. **Error Handling**: Implement robust error handling in AI agents
4. **Resource Management**: Properly manage computational resources for AI algorithms
5. **Testing**: Test AI-ROS integration thoroughly with simulated environments
6. **Logging**: Use appropriate logging levels to track AI decision making

## Next Steps

After completing this chapter, you'll be ready to learn about URDF (Unified Robot Description Format) for humanoid robots in Chapter 6.