---
sidebar_position: 1
---

# Section 1.1: Gazebo Installation and Configuration

## Overview

This section provides detailed instructions for installing and configuring the Gazebo simulation environment. Students will learn to set up Gazebo with appropriate physics engines and verify the installation.

## System Requirements

Before installing Gazebo, ensure your system meets the following requirements:
- Operating System: Ubuntu 22.04 LTS or Windows 10/11 (with WSL2)
- RAM: 8GB minimum, 16GB recommended
- Storage: 10GB available space
- Graphics: OpenGL 3.3 compatible GPU

## Installation Methods

### Method 1: Ubuntu Installation

1. Update package list:
   ```bash
   sudo apt update
   ```

2. Install Gazebo Garden:
   ```bash
   sudo apt install gazebo
   ```

3. Verify installation:
   ```bash
   gazebo --version
   ```

### Method 2: Windows Installation (WSL2)

1. Install WSL2 with Ubuntu 22.04:
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```

2. Once in Ubuntu WSL2, follow the Ubuntu installation steps above.

3. For GUI applications, install an X11 server like VcXsrv or WSLg.

### Method 3: Docker Installation

1. Pull the Gazebo Docker image:
   ```bash
   docker pull gazebosim/gz-harmonic:latest
   ```

2. Run Gazebo in a container:
   ```bash
   docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix gazebosim/gz-harmonic:latest gz sim
   ```

## Configuration

### Physics Engine Selection

For humanoid robot simulation, configure Gazebo to use the DART physics engine:

1. Create or edit the Gazebo configuration file:
   ```bash
   nano ~/.gazebo/config.yaml
   ```

2. Add the following configuration:
   ```yaml
   physics:
     engine: dart
   ```

### Performance Optimization

To optimize performance for humanoid robot simulations:

1. Adjust the update rate in world files:
   ```xml
   <physics type="dart">
     <max_step_size>0.001</max_step_size>
     <real_time_factor>1.0</real_time_factor>
   </physics>
   ```

## Verification

Test the installation by launching a basic simulation:

```bash
gz sim -v 4 shapes.sdf
```

You should see a window with basic geometric shapes falling under gravity.

## Troubleshooting

- If Gazebo fails to launch, ensure your graphics drivers are up to date
- If physics simulation is unstable, try reducing the max_step_size
- For audio issues, install pulseaudio: `sudo apt install pulseaudio`

## Next Steps

After successfully installing and configuring Gazebo, proceed to [Section 1.2: Unity Installation and Configuration](../chapter-1/section-1-2-unity-installation).