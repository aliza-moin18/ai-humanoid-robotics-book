---
sidebar_position: 3
---

# Chapter 11.3: Model Deployment on Edge Hardware

## Introduction

Deploying AI models on edge hardware for robotics applications presents unique challenges and opportunities. Unlike cloud-based solutions, edge deployment enables low-latency, real-time decision making essential for humanoid robot control. This chapter focuses on deploying models trained in NVIDIA Isaac Sim to edge hardware, particularly NVIDIA's Jetson platform optimized for robotics applications.

## Edge Computing in Robotics

Edge computing brings AI processing closer to the robot, reducing latency and enabling real-time responses critical for humanoid robot control. Key advantages include:

1. **Low Latency**: Immediate response to sensor inputs without network delays
2. **Reliability**: Operation independent of network connectivity
3. **Privacy**: Sensitive data processed locally
4. **Bandwidth Efficiency**: Reduced data transmission requirements

## NVIDIA Jetson Platform for Robotics

The NVIDIA Jetson platform is specifically designed for AI at the edge in robotics applications:

### Jetson Hardware Options

1. **Jetson Nano**: Entry-level platform with 472 GFLOPS
2. **Jetson TX2**: Balanced performance with 1.33 TFLOPS
3. **Jetson Xavier NX**: High-performance with 21 TOPS AI performance
4. **Jetson AGX Orin**: Flagship platform with up to 275 TOPS AI performance

### JetPack SDK

JetPack includes all necessary tools for Jetson development:
- Linux for Tegra (L4T) - Ubuntu-based OS
- CUDA toolkit for GPU acceleration
- TensorRT for optimized inference
- Isaac ROS for robotics middleware
- Development tools and libraries

## Model Optimization for Edge Deployment

### TensorRT Optimization

TensorRT is NVIDIA's inference optimizer that can significantly improve model performance on edge devices:

```python
import tensorrt as trt
import numpy as np

def optimize_model_for_jetson(onnx_model_path):
    # Create TensorRT builder
    builder = trt.Builder(trt.Logger(trt.Logger.WARNING))
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, trt.Logger())
    
    # Parse ONNX model
    with open(onnx_model_path, 'rb') as model:
        parser.parse(model.read())
    
    # Configure optimization settings
    config = builder.create_builder_config()
    config.max_workspace_size = 2 << 30  # 2GB
    
    # Build optimized engine
    serialized_engine = builder.build_serialized_network(network, config)
    
    # Save optimized model
    with open("optimized_model.plan", "wb") as f:
        f.write(serialized_engine)
    
    return serialized_engine
```

### Quantization Techniques

Quantization reduces model size and increases inference speed with minimal accuracy loss:

```python
# Example of INT8 quantization with TensorRT
def quantize_model_for_edge(model_path):
    # Create builder and network
    builder = trt.Builder(trt.Logger(trt.Logger.WARNING))
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    config = builder.create_builder_config()
    
    # Enable INT8 calibration
    config.set_flag(trt.BuilderFlag.INT8)
    
    # Set up calibration dataset
    calibrator = trt.IInt8MinMaxCalibrator()
    config.int8_calibrator = calibrator
    
    # Build INT8 engine
    engine = builder.build_engine(network, config)
    
    return engine
```

## Isaac ROS for Edge Deployment

Isaac ROS provides optimized hardware-accelerated perception and navigation nodes:

### Key Isaac ROS Packages

1. **isaac_ros_visual_slam**: GPU-accelerated visual SLAM
2. **isaac_ros_detect_net**: Accelerated object detection
3. **isaac_ros_pose_graph**: Optimized pose estimation
4. **isaac_ros_nitros**: Nitros data format for efficient processing

### Example Isaac ROS Node Configuration

```yaml
# Example Isaac ROS configuration for Jetson deployment
isaac_ros_detect_net:
  ros__parameters:
    # Model configuration
    model_type: 'detectnet'
    model_name: 'resnet18_markerbot'
    engine_file_path: '/models/resnet18_markerbot.trt'
    
    # Input/output configuration
    input_topic: '/camera/color/image_raw'
    output_topic: '/detectnet/detections'
    
    # Performance settings
    inference_fps: 30
    max_batch_size: 1
    
    # TensorRT settings
    input_tensor_names: ['input']
    output_tensor_names: ['output']
    input_binding_names: ['input']
    output_binding_names: ['output']
```

## Deployment Pipeline

### Model Conversion Workflow

1. **Train in Isaac Sim**: Develop and train models in simulation
2. **Export Model**: Convert to ONNX or other deployment format
3. **Optimize for Edge**: Use TensorRT to optimize for Jetson hardware
4. **Deploy to Robot**: Transfer optimized model to edge device
5. **Validate Performance**: Test real-time performance and accuracy

### Docker-Based Deployment

Using Docker containers ensures consistent deployment across different Jetson devices:

```dockerfile
# Dockerfile for Isaac ROS edge deployment
FROM nvcr.io/nvidia/isaac-ros:galactic-ros-base-l4t-r35.2.1

# Install model dependencies
COPY requirements.txt .
RUN pip3 install -r requirements.txt

# Copy optimized model
COPY optimized_model.plan /models/
COPY model_config.yaml /models/

# Copy ROS nodes
COPY src/ /opt/ros_ws/src/
RUN cd /opt/ros_ws && colcon build --packages-select my_robot_nodes

# Set up entrypoint
COPY entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
```

### Deployment Script Example

```python
#!/usr/bin/env python3
# deploy_model.py - Script to deploy model to Jetson device

import subprocess
import sys
import os
import shutil

def deploy_model_to_jetson(model_path, jetson_ip, robot_name):
    """
    Deploy optimized model to Jetson device
    """
    # Optimize model for target Jetson hardware
    optimized_path = optimize_for_jetson(model_path)
    
    # Package model with ROS configuration
    package_path = package_for_deployment(optimized_path, robot_name)
    
    # Transfer to Jetson device
    transfer_to_device(package_path, jetson_ip)
    
    # Restart robot services
    restart_robot_services(jetson_ip)
    
    print(f"Model successfully deployed to {jetson_ip}")

def optimize_for_jetson(model_path):
    """
    Optimize model using TensorRT for Jetson hardware
    """
    # Implementation would use TensorRT to optimize the model
    # for the specific Jetson target hardware
    optimized_path = model_path.replace('.onnx', '_trt.plan')
    
    # Example TensorRT optimization command
    cmd = [
        'trtexec',
        f'--onnx={model_path}',
        f'--saveEngine={optimized_path}',
        '--fp16',  # Use FP16 precision for better performance
        '--workspace=2048'  # 2GB workspace
    ]
    
    subprocess.run(cmd, check=True)
    return optimized_path

def package_for_deployment(model_path, robot_name):
    """
    Package model with necessary configuration files
    """
    package_dir = f"deployment_package_{robot_name}"
    os.makedirs(package_dir, exist_ok=True)
    
    # Copy model
    shutil.copy(model_path, f"{package_dir}/model.plan")
    
    # Copy configuration
    config_path = f"config/{robot_name}_config.yaml"
    shutil.copy(config_path, f"{package_dir}/config.yaml")
    
    # Create deployment manifest
    manifest = {
        "robot_name": robot_name,
        "model_path": "model.plan",
        "config_path": "config.yaml",
        "deployment_date": "2024-01-01"
    }
    
    import json
    with open(f"{package_dir}/manifest.json", 'w') as f:
        json.dump(manifest, f, indent=2)
    
    return package_dir

def transfer_to_device(package_path, jetson_ip):
    """
    Transfer package to Jetson device via SCP
    """
    cmd = [
        'scp', '-r',
        package_path,
        f'jetson@{jetson_ip}:/home/jetson/robot_models/'
    ]
    
    subprocess.run(cmd, check=True)

def restart_robot_services(jetson_ip):
    """
    Restart robot services after model deployment
    """
    ssh_cmd = [
        'ssh', f'jetson@{jetson_ip}',
        'sudo systemctl restart robot-ai.service'
    ]
    
    subprocess.run(ssh_cmd, check=True)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 deploy_model.py <model_path> <jetson_ip>")
        sys.exit(1)
    
    model_path = sys.argv[1]
    jetson_ip = sys.argv[2]
    robot_name = "humanoid_robot"
    
    deploy_model_to_jetson(model_path, jetson_ip, robot_name)
```

## Performance Considerations

### Memory Management

Edge devices have limited memory resources that must be carefully managed:

```python
import psutil
import gc

def monitor_memory_usage():
    """
    Monitor memory usage during model deployment
    """
    memory_percent = psutil.virtual_memory().percent
    print(f"Memory usage: {memory_percent}%")
    
    if memory_percent > 85:
        # Trigger garbage collection
        gc.collect()
        print("Garbage collection triggered due to high memory usage")

def optimize_memory_for_inference(model):
    """
    Optimize model for memory-efficient inference
    """
    # Set model to evaluation mode
    model.eval()
    
    # Disable gradient computation
    for param in model.parameters():
        param.requires_grad = False
    
    # Move model to GPU if available
    import torch
    if torch.cuda.is_available():
        model = model.cuda()
    
    return model
```

### Real-Time Performance

Ensure models meet real-time requirements for robot control:

```python
import time
import threading
from collections import deque

class RealTimeInference:
    def __init__(self, model, target_fps=30):
        self.model = model
        self.target_fps = target_fps
        self.target_interval = 1.0 / target_fps
        self.inference_times = deque(maxlen=100)
        
    def run_inference_with_timing(self, input_data):
        start_time = time.time()
        
        # Perform inference
        with torch.no_grad():
            output = self.model(input_data)
        
        end_time = time.time()
        inference_time = end_time - start_time
        self.inference_times.append(inference_time)
        
        # Log performance metrics
        avg_inference_time = sum(self.inference_times) / len(self.inference_times)
        achieved_fps = 1.0 / avg_inference_time if avg_inference_time > 0 else 0
        
        print(f"Achieved FPS: {achieved_fps:.2f}, Target: {self.target_fps}")
        
        if achieved_fps < self.target_fps * 0.8:  # 80% of target
            print("WARNING: Performance below threshold, consider model optimization")
        
        return output
```

## Troubleshooting Deployment Issues

### Common Problems and Solutions

1. **Model Too Large**: Use quantization or model pruning
2. **Insufficient Memory**: Optimize batch sizes or use model streaming
3. **Performance Issues**: Profile and optimize bottlenecks
4. **Hardware Compatibility**: Verify JetPack version compatibility

### Performance Profiling

```python
# Example of profiling model performance on Jetson
import torch
import torch.profiler

def profile_model_performance(model, sample_input):
    with torch.profiler.profile(
        activities=[torch.profiler.ProfilerActivity.CPU, torch.profiler.ProfilerActivity.CUDA],
        schedule=torch.profiler.schedule(wait=1, warmup=1, active=3, repeat=1),
        on_trace_ready=torch.profiler.tensorboard_trace_handler('./log/profiler'),
        record_shapes=True,
        profile_memory=True,
        with_stack=True
    ) as prof:
        for _ in range(10):
            _ = model(sample_input)
            prof.step()
    
    print(prof.key_averages().table(sort_by="cuda_time_total", row_limit=10))
```

## Assessment Questions

1. What are the key advantages of edge deployment for humanoid robot control?
2. Explain how TensorRT optimization improves model performance on Jetson hardware.
3. What are the main components of the Isaac ROS framework for edge deployment?
4. How can you monitor and optimize memory usage during model deployment?

## Lab Preparation

In the upcoming lab, you will deploy a model trained in Isaac Sim to a Jetson device by:
1. Converting your trained model to TensorRT format
2. Creating a Docker container with your optimized model
3. Deploying the container to a Jetson device
4. Validating real-time performance and accuracy