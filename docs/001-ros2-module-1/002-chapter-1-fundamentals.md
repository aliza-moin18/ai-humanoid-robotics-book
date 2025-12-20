# Chapter 1: ROS 2 Fundamentals

## Overview

This chapter introduces the fundamental concepts of ROS 2, the next-generation Robot Operating System. You'll learn about the architecture, core concepts like nodes, topics, and services, and how they form the foundation of distributed robotic applications.

## Learning Objectives

After completing this chapter, you will be able to:
- Describe the ROS 2 architecture and its advantages over ROS 1
- Understand and implement nodes for computation
- Create and use topics for asynchronous communication
- Implement services for synchronous request-response communication
- Configure and manage parameters in ROS 2 systems

## 1.1 Introduction to ROS 2 Architecture

### What is ROS 2?

ROS 2 (Robot Operating System 2) is the next-generation Robot Operating System designed to address the limitations of ROS 1 and provide a more robust, scalable, and production-ready framework for robotics applications. Unlike ROS 1, which relied on a centralized master node, ROS 2 uses DDS (Data Distribution Service) for communication, providing a decentralized architecture that's more suitable for real-world applications.

### Key Architectural Changes

#### DDS-Based Communication
In ROS 2, communication is based on the Data Distribution Service (DDS) standard. This eliminates the need for a master node and provides:
- Automatic discovery of nodes
- Reliable data transport
- Quality of Service (QoS) policies
- Language and platform independence

#### Improved Security
ROS 2 includes security features from the ground up with:
- Authentication
- Authorization
- Encryption

#### Support for Multiple Middleware
ROS 2 supports different DDS implementations (e.g., Fast DDS, Cyclone DDS, RTI Connext) allowing choice based on requirements.

### Installing ROS 2

The recommended ROS 2 distribution for this module is Humble Hawksbill. Follow the official installation guide for your platform: [ROS 2 Installation](https://docs.ros.org/en/humble/Installation.html)

## 1.2 Nodes and Processes

### Understanding Nodes

A node is the fundamental unit of computation in ROS 2. It's a process that performs computation and communicates with other nodes through topics, services, and actions.

### Creating a Node in Python

Let's create a simple ROS 2 node using the `rclpy` client library:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal_node!')

def main(args=None):
    rclpy.init(args=args)
    
    minimal_node = MinimalNode()
    
    rclpy.spin(minimal_node)
    
    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle

Nodes in ROS 2 have a well-defined lifecycle:
1. Unconfigured
2. Inactive
3. Active
4. Finalized

This lifecycle management allows for better resource management and coordination in complex robotic systems.

## 1.3 Topics and Message Passing

### Understanding Topics

Topics provide asynchronous, many-to-many communication in ROS 2. They use a publish-subscribe pattern where publishers send messages and subscribers receive them without direct connection.

### Creating a Publisher

Here's how to create a simple publisher:

```python
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

### Creating a Subscriber

And here's a corresponding subscriber:

```python
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

## 1.4 Services and Parameters

### Services

Services provide synchronous request-response communication in ROS 2. They're useful when you need to request information or perform an action with guaranteed completion.

Here's an example of a service client:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(f'Result of add_two_ints: {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Parameters

Parameters are named values that can be configured at runtime. They're useful for configuring behavior without recompiling.

Example of working with parameters:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')
        
        # Get parameter value
        my_param = self.get_parameter('my_parameter').value
        self.get_logger().info(f'Parameter value: {my_param}')

def main(args=None):
    rclpy.init(args=args)
    
    parameter_node = ParameterNode()
    
    # Change parameter at runtime
    parameter_node.set_parameters([Parameter('my_parameter', Parameter.Type.STRING, 'new_value')])
    
    rclpy.spin(parameter_node)
    
    parameter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

In this chapter, we covered the foundational concepts of ROS 2:
- Architecture and its advantages over ROS 1
- Nodes as fundamental units of computation
- Topics for asynchronous communication
- Services for synchronous request-response communication
- Parameters for runtime configuration

These concepts form the foundation for all ROS 2 applications. Understanding them thoroughly is essential for the advanced topics covered in subsequent chapters.

## Next Steps

Continue to [Chapter 2: Advanced Communication Patterns](003-chapter-2-advanced-patterns.md) to learn about actions, Quality of Service settings, and launch files.