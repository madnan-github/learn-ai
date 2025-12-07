---
sidebar_position: 3
title: 'Topics and Message Passing'
---

# Topics and Message Passing

This chapter covers the publisher-subscriber communication pattern in ROS 2, which is the most common way for nodes to exchange data in robotic systems.

## What You'll Learn

In this chapter, you'll explore:
- ROS 2 topics and message passing concepts
- Creating publishers and subscribers
- Built-in message types and custom messages
- Quality of Service (QoS) settings
- Debugging topic communication

## Prerequisites

- Completion of Module 1, Chapters 1-2
- Basic Python programming knowledge
- ROS 2 Humble installed on your system

## Understanding Topics

Topics are named buses over which nodes exchange messages. The publisher-subscriber pattern allows for asynchronous communication where:
- Publishers send messages to topics without knowing who receives them
- Subscribers receive messages from topics without knowing who sends them
- Multiple publishers and subscribers can use the same topic

### Topic Communication Characteristics

- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Many-to-many**: Multiple publishers can send to a topic, multiple subscribers can receive from it
- **Typed**: Each topic has a specific message type
- **Named**: Topics follow a naming convention similar to file paths

## Creating Publishers and Subscribers

### Publisher Example

Here's a simple publisher that sends string messages:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

Here's a subscriber that receives the messages:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) Settings

QoS settings allow you to control the reliability and performance characteristics of topic communication:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Create a QoS profile
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,  # or BEST_EFFORT
    history=HistoryPolicy.KEEP_LAST  # or KEEP_ALL
)
```

### Common QoS Settings

- **Reliability**: RELIABLE (ensures delivery) or BEST_EFFORT (faster, no guarantee)
- **History**: KEEP_LAST (store recent messages) or KEEP_ALL (store all messages)
- **Depth**: Number of messages to store in the history

## Hands-on Lab: Temperature Sensor Simulation

In this lab, you'll create a temperature sensor publisher and a data logger subscriber.

### Step 1: Create the Temperature Publisher

Create `temperature_publisher.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperaturePublisher(Node):

    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Temperature publisher started')

    def timer_callback(self):
        msg = Float32()
        # Simulate temperature reading (20-30 degrees Celsius)
        msg.data = round(random.uniform(20.0, 30.0), 2)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Temperature: {msg.data}°C')

def main(args=None):
    rclpy.init(args=args)
    temp_publisher = TemperaturePublisher()

    try:
        rclpy.spin(temp_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        temp_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Create the Temperature Subscriber

Create `temperature_subscriber.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureSubscriber(Node):

    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Temperature subscriber started')

    def temperature_callback(self, msg):
        temp = msg.data
        self.get_logger().info(f'Received temperature: {temp}°C')

        # Alert if temperature is too high
        if temp > 28.0:
            self.get_logger().warn(f'High temperature alert: {temp}°C!')

def main(args=None):
    rclpy.init(args=args)
    temp_subscriber = TemperatureSubscriber()

    try:
        rclpy.spin(temp_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        temp_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Run the Simulation

1. Terminal 1 - Start the publisher:
```bash
source /opt/ros/humble/setup.bash
python3 temperature_publisher.py
```

2. Terminal 2 - Start the subscriber:
```bash
source /opt/ros/humble/setup.bash
python3 temperature_subscriber.py
```

## Built-in Message Types

ROS 2 provides many built-in message types:

- **std_msgs**: Basic data types (String, Int32, Float32, etc.)
- **geometry_msgs**: 3D geometric primitives (Point, Pose, Vector3, etc.)
- **sensor_msgs**: Sensor data (LaserScan, Image, JointState, etc.)
- **nav_msgs**: Navigation messages (Odometry, Path, OccupancyGrid, etc.)

## Debugging Topic Communication

Useful ROS 2 command-line tools for debugging:

```bash
# List all topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /topic_name

# Get information about a topic
ros2 topic info /topic_name

# Publish a message from command line
ros2 topic pub /topic_name std_msgs/String "data: 'Hello'"
```

## Best Practices

1. **Topic Naming**: Use descriptive, consistent names (e.g., `/sensor/temperature`, `/robot/joint_states`)
2. **Message Rate**: Consider the frequency of messages for system performance
3. **QoS Settings**: Choose appropriate settings based on your application requirements
4. **Message Types**: Use appropriate message types for your data
5. **Connection Handling**: Handle connection and disconnection events gracefully

## Next Steps

After completing this chapter, you'll be ready to learn about services and actions in Chapter 4, which provide synchronous and goal-oriented communication patterns respectively.