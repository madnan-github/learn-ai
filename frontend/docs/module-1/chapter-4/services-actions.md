---
sidebar_position: 4
title: 'Services and Actions'
---

# Services and Actions

This chapter covers two important communication patterns in ROS 2: services for synchronous request-response communication and actions for long-running tasks with feedback.

## What You'll Learn

In this chapter, you'll explore:
- ROS 2 services and synchronous communication
- Creating service servers and clients
- ROS 2 actions for long-running tasks
- Creating action servers and clients
- When to use services vs. actions vs. topics

## Prerequisites

- Completion of Module 1, Chapters 1-3
- Basic Python programming knowledge
- ROS 2 Humble installed on your system

## Understanding Services

Services provide synchronous communication between nodes using a request-response pattern:
- Service clients send requests to service servers
- Service servers process requests and return responses
- Communication is blocking - the client waits for a response

### Service Architecture

```
[Client] --(Request)--> [Service Server] --(Response)--> [Client]
```

## Creating Services

### Service Server Example

First, let's create a simple math service that adds two numbers. Create the service definition file `AddTwoInts.srv`:

```
int64 a
int64 b
---
int64 sum
```

Now create the service server:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    minimal_client.get_logger().info(
        f'Result of {sys.argv[1]} + {sys.argv[2]} = {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Understanding Actions

Actions are used for long-running tasks that require:
- Goal specification
- Continuous feedback
- Result reporting
- Cancellation capability

### Action Architecture

Actions have three parts:
1. **Goal**: What the action should do
2. **Feedback**: Progress updates during execution
3. **Result**: Final outcome when complete

## Creating Actions

### Action Server Example

Let's create a Fibonacci action server:

First, create the action definition file `Fibonacci.action`:

```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

Now create the action server:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from example_interfaces.action import Fibonacci
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])

            self.get_logger().info(f'Publishing feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info(f'Returning result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(fibonacci_action_server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        fibonacci_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action Client Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Hands-on Lab: Robot Navigation Service and Action

In this lab, you'll create both a service for immediate robot commands and an action for navigation tasks.

### Step 1: Create the Robot Command Service

Create `robot_command_service.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import random

class RobotCommandService(Node):

    def __init__(self):
        super().__init__('robot_command_service')
        self.srv = self.create_service(
            SetBool,
            'robot_enable',
            self.enable_robot_callback)
        self.robot_enabled = False

    def enable_robot_callback(self, request, response):
        self.robot_enabled = request.data
        if self.robot_enabled:
            self.get_logger().info('Robot enabled')
            response.success = True
            response.message = 'Robot enabled successfully'
        else:
            self.get_logger().info('Robot disabled')
            response.success = True
            response.message = 'Robot disabled successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    robot_service = RobotCommandService()

    try:
        rclpy.spin(robot_service)
    except KeyboardInterrupt:
        pass
    finally:
        robot_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Create the Navigation Action

Create `navigation_action.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time

class NavigationGoal:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

class NavigationResult:
    def __init__(self, success=False, message=""):
        self.success = success
        self.message = message

class NavigationActionServer(Node):

    def __init__(self):
        super().__init__('navigation_action_server')
        # Using a custom action since we don't have a specific navigation action type
        # In a real scenario, you would use nav2_msgs or similar
        self.get_logger().info('Navigation action server initialized')

    def execute_navigation(self, goal):
        self.get_logger().info(f'Navigating to position: ({goal.x}, {goal.y})')

        # Simulate navigation with feedback
        for i in range(10):
            self.get_logger().info(f'Navigating... {i*10}% complete')
            time.sleep(0.5)  # Simulate navigation time

        # Simulate success/failure
        success = random.choice([True, True, True, True, False])  # 80% success rate

        result = NavigationResult()
        if success:
            result.success = True
            result.message = f'Reached goal position ({goal.x}, {goal.y})'
            self.get_logger().info(result.message)
        else:
            result.success = False
            result.message = f'Failed to reach goal position ({goal.x}, {goal.y})'
            self.get_logger().error(result.message)

        return result

def main(args=None):
    rclpy.init(args=args)
    nav_server = NavigationActionServer()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(nav_server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        nav_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Test the Services and Actions

1. Start the service:
```bash
source /opt/ros/humble/setup.bash
python3 robot_command_service.py
```

2. Test the service from another terminal:
```bash
ros2 service call /robot_enable std_srvs/srv/SetBool "{data: true}"
```

## When to Use Each Communication Pattern

### Use Topics When:
- Data needs to be broadcast to multiple subscribers
- Communication can be asynchronous
- Real-time performance is important
- Data is continuously flowing

### Use Services When:
- Request-response pattern is needed
- Operation is relatively quick
- Synchronous communication is acceptable
- Need to perform a specific task once

### Use Actions When:
- Task takes a long time to complete
- Need to provide feedback during execution
- Task might need to be canceled
- Need to track progress of a goal

## Best Practices

1. **Service Design**: Keep service calls short and focused
2. **Action Design**: Use actions for tasks that take more than a few seconds
3. **Error Handling**: Implement proper error handling in both services and actions
4. **Cancellation**: Support cancellation in actions when appropriate
5. **Feedback**: Provide meaningful feedback in actions

## Next Steps

After completing this chapter, you'll be ready to learn about integrating Python agents with ROS controllers using rclpy in Chapter 5.