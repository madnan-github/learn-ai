---
sidebar_position: 2
title: 'ROS 2 Nodes and Architecture'
---

# ROS 2 Nodes and Architecture

This chapter explores the fundamental building blocks of ROS 2 systems - nodes and their architecture. Understanding nodes is crucial for creating distributed robotic applications.

## What You'll Learn

In this chapter, you'll explore:
- The concept of ROS 2 nodes and their role
- Node architecture and communication patterns
- Creating and managing nodes using rclpy
- Node lifecycle and best practices
- Debugging node communication issues

## Prerequisites

- Completion of Module 1, Chapter 1
- Basic Python programming knowledge
- ROS 2 Humble installed on your system

## Understanding ROS 2 Nodes

A ROS 2 node is an executable process that uses ROS 2 client libraries to communicate with other nodes. Nodes are the fundamental building blocks of ROS 2 applications and can:
- Publish messages to topics
- Subscribe to topics to receive messages
- Provide services
- Call services
- Create action servers and clients

### Node Architecture

ROS 2 follows a distributed architecture where nodes can run on the same machine or across a network. The ROS 2 middleware (RMW - ROS Middleware) handles communication between nodes using DDS (Data Distribution Service) implementations.

```
[Node A] -----> [DDS/RMW] -----> [Node B]
```

### Node Naming and Namespaces

Nodes must have unique names within a ROS 2 domain. Namespaces provide a way to organize nodes hierarchically, similar to file system paths.

## Creating Nodes with rclpy

Let's create our first ROS 2 node using Python:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MinimalNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('Minimal node initialized')

def main(args=None):
    rclpy.init(args=args)

    minimal_node = MinimalNode()

    # Spin the node so it continues to run
    rclpy.spin(minimal_node)

    # Clean up
    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle

ROS 2 nodes follow a specific lifecycle:
1. Unconfigured
2. Inactive
3. Active
4. Finalized

This lifecycle allows for better resource management and coordination between nodes.

## Hands-on Lab: Creating Your First Node

In this lab, you'll create a simple node that logs messages at regular intervals.

### Step 1: Create the Node File

Create a file called `timed_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TimedNode(Node):

    def __init__(self):
        super().__init__('timed_node')
        self.counter = 0
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second timer
        self.get_logger().info('Timed node initialized')

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Timer called {self.counter} times')

def main(args=None):
    rclpy.init(args=args)

    timed_node = TimedNode()

    try:
        rclpy.spin(timed_node)
    except KeyboardInterrupt:
        pass
    finally:
        timed_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Make the File Executable

```bash
chmod +x timed_node.py
```

### Step 3: Run the Node

```bash
source /opt/ros/humble/setup.bash
python3 timed_node.py
```

You should see log messages appearing every second.

## Node Communication Patterns

### Publisher-Subscriber Pattern

Nodes communicate through topics using the publisher-subscriber pattern:
- Publishers send messages to topics
- Subscribers receive messages from topics
- Multiple publishers and subscribers can exist for the same topic

### Service-Client Pattern

For request-response communication:
- Services provide functionality
- Clients request specific operations
- Communication is synchronous

### Action Pattern

For long-running operations with feedback:
- Actions have goals, feedback, and results
- Suitable for tasks like navigation or manipulation

## Best Practices

1. **Node Design**: Keep nodes focused on a single responsibility
2. **Error Handling**: Implement proper exception handling
3. **Resource Management**: Clean up resources properly in destructors
4. **Logging**: Use appropriate log levels (info, warn, error, debug)
5. **Parameter Handling**: Use parameters for configuration

## Next Steps

After completing this chapter, you'll be ready to dive deeper into ROS 2 communication patterns by learning about topics and message passing in Chapter 3.