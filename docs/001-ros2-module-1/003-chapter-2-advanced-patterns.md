# Chapter 2: Advanced Communication Patterns

## Overview

This chapter explores advanced communication patterns in ROS 2 that go beyond the basic publish-subscribe and service models. You'll learn about actions for long-running tasks with feedback, Quality of Service (QoS) policies for controlling communication behavior, and launch systems for managing complex robotic applications.

## Learning Objectives

After completing this chapter, you will be able to:
- Implement and use actions for long-running tasks with feedback
- Apply Quality of Service policies to control communication behavior
- Create and use launch files for system composition
- Effectively manage parameters in complex systems

## 2.1 Actions for Complex Tasks

### Understanding Actions

Actions are a more sophisticated communication pattern in ROS 2 designed for long-running tasks that require feedback and cancellation capabilities. Unlike services, which are blocking and synchronous, actions are non-blocking and provide continuous feedback during execution.

### Action Structure

An action consists of three message types:
- **Goal**: The request sent to the action server
- **Feedback**: Messages sent periodically during action execution
- **Result**: The final outcome of the action

### Creating an Action Client

Here's an example of an action client:

```python
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        # Send the goal and register callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
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

### Creating an Action Server

And here's a corresponding action server:

```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: {goal_handle.request.order}')

        # Create a result message
        feedback_msg = Fibonacci.Feedback()
        result = Fibonacci.Result()

        # Initialize sequence
        fibonacci_sequence = [0, 1]
        
        # Publish feedback as the sequence progresses
        for i in range(1, goal_handle.request.order):
            # Check if there was a request to cancel the action
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.sequence = fibonacci_sequence
                return result

            # Update sequence
            fibonacci_sequence.append(fibonacci_sequence[i] + fibonacci_sequence[i-1])
            
            # Publish feedback
            feedback_msg.partial_sequence = fibonacci_sequence
            goal_handle.publish_feedback(feedback_msg)
            
            # Sleep to simulate work
            time.sleep(1)

        # Finish the action
        goal_handle.succeed()
        result.sequence = fibonacci_sequence
        
        self.get_logger().info(f'Sending result: {result.sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)

if __name__ == '__main__':
    main()
```

## 2.2 Quality of Service (QoS) Settings

### Understanding QoS

Quality of Service (QoS) policies in ROS 2 allow you to control how messages are transmitted between publishers and subscribers. This is particularly important for real-time applications where you need guarantees about message delivery.

### Key QoS Policies

1. **Reliability**: Defines whether messages should be reliably delivered
   - `RELIABLE`: Every message will be delivered
   - `BEST_EFFORT`: Messages may be lost, but will be delivered quickly

2. **Durability**: Controls how messages are preserved for late-joining subscribers
   - `TRANSIENT_LOCAL`: Messages are stored for late joiners
   - `VOLATILE`: Messages are not stored for late joiners

3. **History**: Controls how many messages to store
   - `KEEP_LAST`: Store the N most recent messages
   - `KEEP_ALL`: Store all messages (limited by resource availability)

4. **Depth**: Number of messages to store when using KEEP_LAST history

### Using QoS Profiles

Example of using QoS with different settings:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')
        
        # Create a QoS profile with custom settings
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST)
        
        self.publisher_ = self.create_publisher(String, 'qos_chatter', qos_profile)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'QoS message: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

class QoSSubscriber(Node):
    def __init__(self):
        super().__init__('qos_subscriber')
        
        # Create a QoS profile with matching settings to the publisher
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST)
        
        self.subscription = self.create_subscription(
            String,
            'qos_chatter',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'QoS received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    publisher_node = QoSPublisher()
    subscriber_node = QoSSubscriber()

    # Run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(publisher_node)
    executor.add_node(subscriber_node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        publisher_node.destroy_node()
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2.3 Launch Files and System Composition

### Introduction to Launch Files

Launch files in ROS 2 allow you to start multiple nodes with a single command, making system composition much easier. They're written in Python and provide powerful features for managing complex robotic systems.

### Creating Launch Files

Example launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='talker',
            parameters=[
                {'param_name': 'param_value'}
            ],
            remappings=[
                ('original_topic', 'remapped_topic')
            ]
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener',
            parameters=[
                {'param_name': 'param_value'}
            ]
        )
    ])
```

### Advanced Launch Concepts

Launch files can also include:
- Conditional execution
- Events and event handlers
- Parameter files
- YAML configuration files

### Parameter Files

You can also manage parameters using YAML files:

```yaml
talker:
  ros__parameters:
    param_name: param_value
    frequency: 1.0

listener:
  ros__parameters:
    param_name: param_value
```

Then load the parameter file in the launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_py',
            executable='talker',
            name='talker',
            parameters=['path/to/params.yaml']
        )
    ])
```

## 2.4 Parameter Management (Expanded)

### Advanced Parameter Concepts

In addition to basic parameter usage, ROS 2 offers more sophisticated parameter management features:

#### Parameter Descriptors

You can describe parameters with additional metadata:

```python
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

class ParameterDescriptorNode(Node):
    def __init__(self):
        super().__init__('parameter_descriptor_node')
        
        # Declare parameter with descriptor
        descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='A parameter with description',
            additional_constraints='Must be a valid string',
            read_only=False,
            floating_point_range=[],
            integer_range=[]
        )
        
        self.declare_parameter('described_param', 'default_value', descriptor)
```

#### Parameter Callbacks

You can register callbacks to handle parameter changes:

```python
from rclpy.parameter import Parameter

class ParameterCallbackNode(Node):
    def __init__(self):
        super().__init__('parameter_callback_node')
        
        self.declare_parameter('changeable_param', 10)
        
        # Register callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'changeable_param' and param.value > 100:
                return SetParametersResult(successful=False, reason='Parameter too large')
        return SetParametersResult(successful=True)
```

## Summary

This chapter covered advanced communication patterns in ROS 2:
- Actions for long-running tasks with feedback and cancellation
- Quality of Service policies for controlling communication behavior
- Launch files for system composition
- Advanced parameter management techniques

These advanced patterns are crucial for developing robust, real-world robotic applications that require sophisticated communication between components.

## Next Steps

Continue to [Chapter 3: Robot Description and Modeling](004-chapter-3-robot-modeling.md) to learn about URDF (Unified Robot Description Format) and robot modeling.