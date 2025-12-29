---
sidebar_position: 1
---

# Introduction to ROS 2 Architecture

## Overview

The Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Unlike traditional operating systems, ROS 2 is not an actual operating system but rather a middleware framework that provides services designed for a heterogeneous computer cluster. It includes hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

## Key Concepts

### Nodes
Nodes are the fundamental unit of execution in ROS 2. They are processes that perform computation and communicate with other nodes through messages. In ROS 2, nodes are implemented as objects that can be contained within a process (a single executable can contain multiple nodes).

### Topics and Messages
Topics enable asynchronous communication between nodes using a publish/subscribe model. Nodes can publish messages to a topic, and other nodes can subscribe to that topic to receive messages. Messages are data structures that are exchanged between nodes.

### Services
Services provide synchronous communication between nodes using a request/response model. A node sends a request to another node and waits for a response.

### Actions
Actions are similar to services but are designed for long-running tasks. They allow clients to send a goal to an action server, receive feedback during the goal's execution, and get a result when the goal is completed.

### Parameters
Parameters are configuration values that can be set at runtime and accessed by nodes. They can be global to the system or specific to individual nodes.

## ROS 2 Architecture

ROS 2 uses a DDS (Data Distribution Service) implementation for its communication layer. This provides:

- **Decentralized**: No central master process like in ROS 1
- **Real-time support**: Better support for real-time systems
- **Multi-platform**: Support for various operating systems
- **Security**: Built-in security capabilities

## Why ROS 2?

ROS 2 addresses several limitations of ROS 1:

- **Real-time support**: Better support for real-time systems
- **Multi-robot systems**: Improved support for multiple robots
- **Production environments**: More suitable for deployment in production
- **Cross-platform support**: Better support for Windows and macOS
- **Security**: Built-in security capabilities
- **DDS middleware**: Pluggable middleware for different communication needs

## Next Steps

In the following sections, we'll explore each of these concepts in detail and learn how to implement them using the Python client library (rclpy).