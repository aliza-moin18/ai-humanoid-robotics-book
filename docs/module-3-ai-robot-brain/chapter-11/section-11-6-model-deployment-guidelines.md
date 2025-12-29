---
sidebar_position: 6
---

# Chapter 11.6: Model Deployment Guidelines for Jetson Hardware

## Introduction

Deploying AI models on NVIDIA Jetson hardware for robotics applications requires careful consideration of hardware constraints, performance requirements, and optimization techniques. This chapter provides comprehensive guidelines for successfully deploying models trained in Isaac Sim to Jetson platforms, ensuring optimal performance and real-time operation.

## Jetson Hardware Overview

### Available Platforms

NVIDIA Jetson offers several platforms optimized for different robotics applications:

1. **Jetson Nano**: 472 GFLOPS, 4GB/2GB LPDDR4, ideal for entry-level robotics
2. **Jetson TX2**: 1.33 TFLOPS, 8GB LPDDR4, balanced performance for mid-tier robots
3. **Jetson Xavier NX**: 21 TOPS AI performance, 8GB LPDDR4, high-performance edge AI
4. **Jetson AGX Xavier**: 32 TOPS AI performance, 32GB LPDDR4, professional robotics
5. **Jetson AGX Orin**: 275 TOPS AI performance, 64GB LPDDR4, flagship platform
6. **Jetson Orin NX/Nano**: 70-100 TOPS, 4-8GB LPDDR5, cost-optimized options

### Hardware Specifications for Robotics

When deploying models to Jetson hardware, consider these key specifications:

- **Compute Capability**: Affects model inference speed
- **Memory**: Influences model size and batch processing capabilities
- **Power Consumption**: Critical for mobile robots with limited battery life
- **Thermal Design**: Affects sustained performance under load
- **Connectivity**: Required for sensor integration and communication

## Model Optimization for Jetson

### TensorRT Optimization

TensorRT is NVIDIA's inference optimizer, essential for achieving real-time performance on Jetson:

```python
import tensorrt as trt
import numpy as np
import pycuda.driver as cuda
import pycuda.autoinit

def optimize_model_for_jetson(model_path, precision="fp16"):
    """
    Optimize model using TensorRT for Jetson deployment
    """
    # Create TensorRT builder
    builder = trt.Builder(trt.Logger(trt.Logger.WARNING))
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    config = builder.create_builder_config()
    
    # Parse the model (assuming ONNX format)
    parser = trt.OnnxParser(network, trt.Logger())
    
    with open(model_path, 'rb') as model_file:
        if not parser.parse(model_file.read()):
            print("ERROR: Failed to parse the ONNX file")
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            return None
    
    # Set optimization profiles
    profile = builder.create_optimization_profile()
    input_name = network.get_input(0).name
    profile.set_shape(input_name, (1, *network.get_input(0).shape[1:]),    # min shape
                      (4, *network.get_input(0).shape[1:]),                 # opt shape  
                      (8, *network.get_input(0).shape[1:]))                 # max shape
    config.add_optimization_profile(profile)
    
    # Set precision based on requirements
    if precision == "fp16":
        config.set_flag(trt.BuilderFlag.FP16)
    elif precision == "int8":
        config.set_flag(trt.BuilderFlag.INT8)
        # Set up INT8 calibration here if needed
    
    # Set workspace size (adjust based on Jetson model)
    if "orin" in model_path.lower():
        config.max_workspace_size = 2 << 30  # 2GB for Orin
    else:
        config.max_workspace_size = 1 << 30  # 1GB for other models
    
    # Build the engine
    serialized_engine = builder.build_serialized_network(network, config)
    
    if serialized_engine is None:
        print("ERROR: Failed to build the TensorRT engine")
        return None
    
    # Save optimized model
    engine_path = model_path.replace(".onnx", f"_trt_{precision}.engine")
    with open(engine_path, "wb") as f:
        f.write(serialized_engine)
    
    print(f"Optimized model saved to {engine_path}")
    return engine_path
```

### Model Quantization

Quantization reduces model size and increases inference speed with minimal accuracy loss:

```python
import torch
import torch_tensorrt

def quantize_model_for_jetson(model, sample_input, precision="fp16"):
    """
    Quantize model for Jetson deployment using Torch-TensorRT
    """
    # Trace the model
    traced_model = torch.jit.trace(model.eval(), sample_input)
    
    # Compile with Torch-TensorRT
    if precision == "fp16":
        precision_set = {torch.float, torch.half}
    elif precision == "int8":
        precision_set = {torch.int8}
    else:
        precision_set = {torch.float}
    
    # Calculate workspace size based on Jetson model
    jetson_workspace_size = 1 << 30  # 1GB default
    if "orin" in precision.lower() or "xavier" in precision.lower():
        jetson_workspace_size = 2 << 30  # 2GB for higher-end models
    
    optimized_model = torch_tensorrt.compile(
        traced_model,
        inputs=[
            torch_tensorrt.Input(
                min_shape=[1, *sample_input.shape[1:]],
                opt_shape=[4, *sample_input.shape[1:]],
                max_shape=[8, *sample_input.shape[1:]]
            )
        ],
        enabled_precisions=precision_set,
        workspace_size=jetson_workspace_size,
        debug=True
    )
    
    return optimized_model

def apply_int8_quantization_with_calibration(model, calibration_data_loader):
    """
    Apply INT8 quantization with calibration for maximum optimization
    """
    import torch.quantization as quantization
    
    # Set model to evaluation mode
    model.eval()
    
    # Specify quantization configuration
    model.qconfig = quantization.get_default_qconfig('fbgemm')
    
    # Prepare model for quantization
    quantization.prepare(model, inplace=True)
    
    # Run calibration data through model
    with torch.no_grad():
        for data, target in calibration_data_loader:
            model(data)
    
    # Convert to quantized model
    quantization.convert(model, inplace=True)
    
    return model
```

## JetPack SDK and Isaac ROS Integration

### JetPack SDK Components

JetPack includes all necessary tools for Jetson development:

```yaml
# JetPack components relevant for robotics
JetPack Components:
  - Linux for Tegra (L4T): Ubuntu-based OS optimized for Jetson
  - CUDA: Parallel computing platform and programming model
  - TensorRT: Deep learning inference optimizer
  - Isaac ROS: Hardware-accelerated ROS 2 packages
  - Isaac Sim: Robotics simulation application
  - DeepStream: Streaming analytics toolkit
  - VisionWorks: Computer vision library
```

### Isaac ROS for Hardware Acceleration

Isaac ROS provides optimized hardware-accelerated nodes for common robotics tasks:

```python
# Example Isaac ROS node configuration
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from isaac_ros_visual_slam import VisualSLAMNode

class OptimizedRobotController(Node):
    def __init__(self):
        super().__init__('optimized_robot_controller')
        
        # Subscribe to camera feed
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10
        )
        
        # Initialize Isaac ROS Visual SLAM node
        self.visual_slam = VisualSLAMNode(
            node_name='visual_slam',
            enable_rectification=True,
            enable_fisheye=False
        )
        
        # Initialize your optimized model
        self.load_optimized_model()
    
    def load_optimized_model(self):
        """
        Load the TensorRT optimized model
        """
        import tensorrt as trt
        import pycuda.driver as cuda
        
        # Load TensorRT engine
        with open('optimized_model.engine', 'rb') as f:
            engine_data = f.read()
        
        # Create runtime and deserialize engine
        self.runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
        self.engine = self.runtime.deserialize_cuda_engine(engine_data)
        self.context = self.engine.create_execution_context()
        
        # Allocate buffers
        self.allocate_buffers()
    
    def allocate_buffers(self):
        """
        Allocate input and output buffers for TensorRT inference
        """
        # Calculate buffer sizes
        for idx in range(self.engine.num_bindings):
            binding_shape = self.engine.get_binding_shape(idx)
            size = trt.volume(binding_shape) * self.engine.max_batch_size * 4  # 4 bytes per float32
            self.cuda_buffer = cuda.mem_alloc(size)
    
    def camera_callback(self, msg):
        """
        Process camera input with optimized model
        """
        # Convert ROS image to tensor
        input_tensor = self.ros_image_to_tensor(msg)
        
        # Run optimized inference
        output = self.run_inference(input_tensor)
        
        # Process output for robot control
        self.process_control_output(output)
```

## Deployment Pipeline

### Model Conversion Workflow

```python
#!/usr/bin/env python3
# deployment_pipeline.py

import os
import subprocess
import shutil
import sys

def convert_model_for_jetson(input_model_path, target_jetson_model, output_dir):
    """
    Complete model conversion pipeline for Jetson deployment
    """
    print(f"Starting model conversion for {target_jetson_model}")
    
    # Step 1: Convert to ONNX if needed
    onnx_path = convert_to_onnx_if_needed(input_model_path)
    
    # Step 2: Optimize with TensorRT
    optimized_path = optimize_with_tensorrt(onnx_path, target_jetson_model)
    
    # Step 3: Package for deployment
    package_path = package_for_deployment(optimized_path, target_jetson_model, output_dir)
    
    print(f"Model conversion completed. Package available at {package_path}")
    return package_path

def convert_to_onnx_if_needed(model_path):
    """
    Convert model to ONNX format if it's not already in ONNX
    """
    if model_path.endswith('.onnx'):
        return model_path
    
    # Convert PyTorch model to ONNX
    import torch
    
    # Load model
    model = torch.load(model_path)
    model.eval()
    
    # Create sample input (adjust dimensions as needed)
    sample_input = torch.randn(1, 3, 224, 224)  # Adjust based on your model
    
    # Convert to ONNX
    onnx_path = model_path.replace('.pt', '.onnx').replace('.pth', '.onnx')
    torch.onnx.export(
        model,
        sample_input,
        onnx_path,
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={
            'input': {0: 'batch_size'},
            'output': {0: 'batch_size'}
        }
    )
    
    print(f"Model converted to ONNX: {onnx_path}")
    return onnx_path

def optimize_with_tensorrt(onnx_path, target_jetson_model):
    """
    Optimize model using TensorRT for the specific Jetson model
    """
    import tensorrt as trt
    
    # Determine precision based on Jetson model
    if "nano" in target_jetson_model.lower():
        precision = "fp16"  # FP16 for lower-end models
    else:
        precision = "fp16"  # Usually FP16 for optimal balance
    
    return optimize_model_for_jetson(onnx_path, precision)

def package_for_deployment(optimized_model_path, jetson_model, output_dir):
    """
    Package optimized model with necessary configuration files
    """
    # Create deployment package directory
    package_dir = os.path.join(output_dir, f"deployment_package_{jetson_model.replace(' ', '_').lower()}")
    os.makedirs(package_dir, exist_ok=True)
    
    # Copy optimized model
    model_filename = os.path.basename(optimized_model_path)
    shutil.copy(optimized_model_path, os.path.join(package_dir, model_filename))
    
    # Create model configuration
    config_content = f"""
# Model Configuration for {jetson_model}
model_file: {model_filename}
input_shape: [1, 3, 224, 224]  # Adjust based on your model
output_shape: [1, 1000]        # Adjust based on your model
precision: fp16
max_batch_size: 1
workspace_size: {get_workspace_size(jetson_model)}
"""
    
    config_path = os.path.join(package_dir, "model_config.yaml")
    with open(config_path, 'w') as f:
        f.write(config_content)
    
    # Create deployment script
    deploy_script = f"""#!/bin/bash
# Deploy script for {jetson_model}

set -e  # Exit on any error

# Configuration
MODEL_FILE="{model_filename}"
CONFIG_FILE="model_config.yaml"
TARGET_DIR="/opt/robot/models"

# Create target directory
sudo mkdir -p $TARGET_DIR

# Copy model and configuration
sudo cp $MODEL_FILE $TARGET_DIR/
sudo cp $CONFIG_FILE $TARGET_DIR/

# Set permissions
sudo chmod 644 $TARGET_DIR/$MODEL_FILE
sudo chmod 644 $TARGET_DIR/$CONFIG_FILE

echo "Model deployed to $TARGET_DIR"
echo "Restarting robot services..."
sudo systemctl restart robot-ai.service

echo "Deployment completed successfully"
"""
    
    deploy_script_path = os.path.join(package_dir, "deploy.sh")
    with open(deploy_script_path, 'w') as f:
        f.write(deploy_script)
    
    # Make deploy script executable
    os.chmod(deploy_script_path, 0o755)
    
    # Create README
    readme_content = f"""
# Deployment Package for {jetson_model}

This package contains an optimized model for deployment on {jetson_model} hardware.

## Contents
- {model_filename}: TensorRT optimized model
- model_config.yaml: Model configuration
- deploy.sh: Deployment script

## Deployment Instructions

1. Transfer this package to your Jetson device
2. Run the deployment script: `./deploy.sh`
3. Verify the deployment: `systemctl status robot-ai.service`

## Hardware Requirements
- {jetson_model}
- JetPack SDK (version matching hardware)
- Sufficient storage for model files

For troubleshooting, see the robotics documentation.
"""
    
    readme_path = os.path.join(package_dir, "README.md")
    with open(readme_path, 'w') as f:
        f.write(readme_content)
    
    return package_dir

def get_workspace_size(jetson_model):
    """
    Get appropriate workspace size based on Jetson model
    """
    if "orin" in jetson_model.lower():
        return "2147483648"  # 2GB in bytes
    elif "xavier" in jetson_model.lower():
        return "1073741824"  # 1GB in bytes
    else:
        return "536870912"   # 512MB for lower-end models

# Example usage
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 deployment_pipeline.py <input_model_path> <target_jetson_model>")
        print("Example: python3 deployment_pipeline.py my_model.pt 'Jetson AGX Orin'")
        sys.exit(1)
    
    input_model = sys.argv[1]
    jetson_model = sys.argv[2]
    
    convert_model_for_jetson(input_model, jetson_model, "./deployment_packages")
```

### Docker-Based Deployment

Using Docker containers ensures consistent deployment across different Jetson devices:

```dockerfile
# Dockerfile for Isaac ROS and model deployment
FROM nvcr.io/nvidia/isaac-ros:galactic-ros-base-l4t-r35.2.1

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV NVIDIA_REQUIRE_CUDA="cuda>=11.4 brand=tesla,driver>=470,driver<471"

# Install Python dependencies
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Install additional dependencies for robotics
RUN apt-get update && apt-get install -y \
    ros-humble-vision-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Copy optimized model
COPY optimized_model.plan /models/
COPY model_config.yaml /models/

# Copy Isaac ROS nodes
COPY src/ /opt/ros_ws/src/
RUN cd /opt/ros_ws && \
    export PYTHONPATH=/opt/conda/lib/python3.8/site-packages:$PYTHONPATH && \
    colcon build --packages-select my_robot_nodes

# Set up runtime environment
ENV ROS_DOMAIN_ID=0
ENV RCUTILS_COLORIZED_OUTPUT=1

# Copy launch files
COPY launch/ /opt/launch/

# Set up entrypoint
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

```bash
#!/bin/bash
# entrypoint.sh - Entry point for Isaac ROS container

# Source ROS environment
source /opt/ros/humble/setup.bash
source /opt/ros_ws/install/setup.bash

# Set up device permissions for Jetson hardware
if [ -e /dev/video0 ]; then
    chmod 666 /dev/video0
fi

# Run the main application
exec "$@"
```

## Performance Optimization

### Memory Management

Efficient memory management is crucial for Jetson deployment:

```python
import torch
import gc

def optimize_memory_usage():
    """
    Optimize memory usage on Jetson device
    """
    # Clear CUDA cache
    if torch.cuda.is_available():
        torch.cuda.empty_cache()
        torch.cuda.synchronize()
    
    # Force garbage collection
    gc.collect()
    
    # Monitor memory usage
    if torch.cuda.is_available():
        memory_allocated = torch.cuda.memory_allocated() / 1024**3  # GB
        memory_reserved = torch.cuda.memory_reserved() / 1024**3    # GB
        
        print(f"CUDA Memory - Allocated: {memory_allocated:.2f}GB, "
              f"Reserved: {memory_reserved:.2f}GB")

class MemoryEfficientModelRunner:
    def __init__(self, model_path):
        self.model_path = model_path
        self.model = None
        self.load_model()
    
    def load_model(self):
        """
        Load model with memory-efficient settings
        """
        # Load model to CPU first to avoid memory spikes
        self.model = torch.jit.load(self.model_path, map_location='cpu')
        
        # Move to GPU if available
        if torch.cuda.is_available():
            self.model = self.model.cuda()
            self.model = self.model.eval()  # Set to evaluation mode
        
        # Disable gradients for inference
        for param in self.model.parameters():
            param.requires_grad = False
    
    def run_inference(self, input_tensor):
        """
        Run inference with memory-efficient approach
        """
        with torch.no_grad():
            if torch.cuda.is_available():
                input_tensor = input_tensor.cuda()
            
            # Run inference
            output = self.model(input_tensor)
            
            # Move output back to CPU to free GPU memory
            if torch.cuda.is_available():
                output = output.cpu()
        
        # Clear cache after inference
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        
        return output
```

### Real-Time Performance

Ensure models meet real-time requirements for robot control:

```python
import time
import threading
from collections import deque

class RealTimeModelInference:
    def __init__(self, model_path, target_fps=30):
        self.target_fps = target_fps
        self.target_interval = 1.0 / target_fps
        self.model_runner = MemoryEfficientModelRunner(model_path)
        self.inference_times = deque(maxlen=100)
        self.latency_threshold = self.target_interval * 2  # 2x tolerance
        
    def run_inference_with_timing(self, input_data):
        """
        Run inference while monitoring timing constraints
        """
        start_time = time.time()
        
        # Perform inference
        output = self.model_runner.run_inference(input_data)
        
        end_time = time.time()
        inference_time = end_time - start_time
        self.inference_times.append(inference_time)
        
        # Check if we're meeting timing requirements
        if inference_time > self.latency_threshold:
            print(f"WARNING: Inference time {inference_time:.3f}s "
                  f"exceeds threshold {self.latency_threshold:.3f}s")
        
        # Calculate performance metrics
        avg_inference_time = sum(self.inference_times) / len(self.inference_times)
        achieved_fps = 1.0 / avg_inference_time if avg_inference_time > 0 else 0
        
        if achieved_fps < self.target_fps * 0.8:  # 80% of target
            print(f"WARNING: Achieved FPS {achieved_fps:.2f} below "
                  f"80% of target {self.target_fps}")
        
        return output, inference_time

def optimize_for_real_time_performance(model_runner, sample_input):
    """
    Optimize model for real-time performance
    """
    # Warm up the model
    for _ in range(10):
        _ = model_runner.run_inference(sample_input)
    
    # Profile performance
    timing_analyzer = RealTimeModelInference(model_runner.model_path)
    
    # Run several inferences to get average timing
    for _ in range(50):
        _, _ = timing_analyzer.run_inference(sample_input)
    
    # Calculate average inference time
    avg_time = sum(timing_analyzer.inference_times) / len(timing_analyzer.inference_times)
    print(f"Average inference time: {avg_time:.4f}s "
          f"({1/avg_time:.2f} FPS)")
    
    return timing_analyzer
```

## Troubleshooting Deployment Issues

### Common Problems and Solutions

1. **Model Too Large**: Use quantization or model pruning
2. **Insufficient Memory**: Optimize batch sizes or use model streaming
3. **Performance Issues**: Profile and optimize bottlenecks
4. **Hardware Compatibility**: Verify JetPack version compatibility

```python
def troubleshoot_deployment_issues():
    """
    Diagnose common deployment issues on Jetson
    """
    issues = []
    
    # Check memory
    import psutil
    memory_percent = psutil.virtual_memory().percent
    if memory_percent > 90:
        issues.append("High memory usage detected (>90%)")
    
    # Check disk space
    disk_usage = psutil.disk_usage('/')
    disk_percent = (disk_usage.used / disk_usage.total) * 100
    if disk_percent > 90:
        issues.append("Low disk space (<10% free)")
    
    # Check GPU status
    try:
        import subprocess
        gpu_status = subprocess.run(['nvidia-smi'], 
                                   capture_output=True, text=True)
        if 'error' in gpu_status.stderr.lower():
            issues.append("GPU not accessible or driver issue")
    except:
        issues.append("Could not check GPU status")
    
    # Check JetPack version
    try:
        jetpack_version = subprocess.run(['head', '-1', '/etc/nv_tegra_release'], 
                                        capture_output=True, text=True)
        if not jetpack_version.stdout:
            issues.append("Could not determine JetPack version")
    except:
        issues.append("Could not check JetPack version")
    
    if issues:
        print("Potential issues detected:")
        for issue in issues:
            print(f"  - {issue}")
    else:
        print("No obvious issues detected")
    
    return issues

def optimize_model_size(model, target_size_mb):
    """
    Optimize model to fit within target size constraints
    """
    import torch.nn.utils.prune as prune
    
    # Calculate current model size
    param_size = 0
    for param in model.parameters():
        param_size += param.nelement() * param.element_size()
    buffer_size = 0
    for buffer in model.buffers():
        buffer_size += buffer.nelement() * buffer.element_size()
    
    total_size_mb = (param_size + buffer_size) / 1024**2
    print(f"Current model size: {total_size_mb:.2f} MB")
    
    if total_size_mb <= target_size_mb:
        print("Model already fits within target size")
        return model
    
    # Calculate required pruning ratio
    target_ratio = 1 - (target_size_mb / total_size_mb)
    
    if target_ratio > 0.5:
        print("WARNING: Required pruning ratio is high, may affect accuracy")
    
    # Apply pruning
    for name, module in model.named_modules():
        if isinstance(module, torch.nn.Linear):
            prune.l1_unstructured(module, name='weight', amount=target_ratio)
            # Remove pruning reparametrization to make it permanent
            prune.remove(module, 'weight')
    
    # Recalculate size after pruning
    param_size = 0
    for param in model.parameters():
        param_size += param.nelement() * param.element_size()
    new_total_size_mb = param_size / 1024**2
    print(f"Model size after pruning: {new_total_size_mb:.2f} MB")
    
    return model
```

## Best Practices

### Deployment Best Practices

1. **Test on Target Hardware**: Always validate performance on the actual Jetson device
2. **Monitor Resource Usage**: Continuously monitor CPU, GPU, and memory usage
3. **Implement Fallbacks**: Have backup plans if optimized model fails
4. **Version Control**: Keep track of model versions and hardware configurations
5. **Security**: Ensure deployment packages are secure and verified

### Performance Monitoring

```python
import psutil
import time
import threading

class JetsonPerformanceMonitor:
    def __init__(self):
        self.stats = {
            'cpu_percent': [],
            'memory_percent': [],
            'gpu_util': [],
            'temperature': []
        }
        self.monitoring = False
        self.monitor_thread = None
    
    def start_monitoring(self):
        """Start performance monitoring in a separate thread"""
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.start()
    
    def stop_monitoring(self):
        """Stop performance monitoring"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join()
    
    def _monitor_loop(self):
        """Main monitoring loop"""
        while self.monitoring:
            # CPU usage
            self.stats['cpu_percent'].append(psutil.cpu_percent())
            
            # Memory usage
            self.stats['memory_percent'].append(psutil.virtual_memory().percent)
            
            # GPU usage (simplified - actual implementation would use nvidia-ml-py)
            try:
                import subprocess
                gpu_info = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu', 
                                          '--format=csv,noheader,nounits'], 
                                         capture_output=True, text=True)
                gpu_util = int(gpu_info.stdout.strip()) if gpu_info.stdout.strip().isdigit() else 0
                self.stats['gpu_util'].append(gpu_util)
            except:
                self.stats['gpu_util'].append(0)  # Default if nvidia-smi fails
            
            # Temperature (simplified)
            try:
                temp_info = subprocess.run(['cat', '/sys/class/thermal/thermal_zone0/temp'], 
                                          capture_output=True, text=True)
                temp = int(temp_info.stdout.strip()) / 1000.0  # Convert from millidegrees
                self.stats['temperature'].append(temp)
            except:
                self.stats['temperature'].append(0)  # Default if temp reading fails
            
            time.sleep(1)  # Monitor every second
    
    def get_average_stats(self):
        """Get average performance statistics"""
        avg_stats = {}
        for key, values in self.stats.items():
            if values:
                avg_stats[key] = sum(values) / len(values)
            else:
                avg_stats[key] = 0
        return avg_stats

# Usage example
monitor = JetsonPerformanceMonitor()
monitor.start_monitoring()

# Run your inference code here
# ...

monitor.stop_monitoring()
avg_stats = monitor.get_average_stats()
print(f"Average stats: {avg_stats}")
```

## Assessment Questions

1. What are the key differences between Jetson hardware platforms for robotics applications?
2. Explain the process of optimizing a model using TensorRT for Jetson deployment.
3. How does Isaac ROS enhance model deployment on Jetson hardware?
4. What techniques can be used to monitor and optimize memory usage during deployment?
5. What are the best practices for ensuring real-time performance on Jetson devices?

## Lab Preparation

Before deploying your models to Jetson hardware:
1. Verify JetPack SDK installation on target device
2. Test model inference in simulation environment
3. Profile model performance and memory requirements
4. Prepare deployment package with all dependencies
5. Have a rollback plan in case of deployment issues