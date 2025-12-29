# ROS 2 Environment Setup

## Overview

This guide will help you set up your ROS 2 development environment. We'll be using ROS 2 Humble Hawksbill, which is an LTS (Long Term Support) version suitable for learning and development.

## System Requirements

- Ubuntu 22.04 (Jammy Jellyfish) or Windows 10/11 with WSL2
- At least 4GB RAM (8GB recommended)
- At least 20GB free disk space
- Internet connection for package downloads

## Installation Options

### Option 1: Native Ubuntu Installation (Recommended)

1. Update your system packages:
   ```bash
   sudo apt update && sudo apt upgrade
   ```

2. Set locale:
   ```bash
   locale-gen en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

3. Add the ROS 2 repository:
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

4. Install ROS 2 Humble:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

5. Install colcon build tools:
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

6. Install ROS dependencies:
   ```bash
   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   ```

### Option 2: Using WSL2 (Windows)

1. Install WSL2 with Ubuntu 22.04
2. Follow the same steps as Option 1 within your WSL2 environment

### Option 3: Using Docker (Alternative)

1. Install Docker from https://docs.docker.com/get-docker/
2. Pull the official ROS 2 Humble image:
   ```bash
   docker pull ros:humble
   ```

## Environment Setup

After installation, add the following line to your `~/.bashrc` file:
```bash
source /opt/ros/humble/setup.bash
```

Then reload your bash configuration:
```bash
source ~/.bashrc
```

## Verification

To verify your installation, run:
```bash
ros2 --version
```

You should see the version of ROS 2 installed (e.g., `ros2 foxy`, `ros2 galactic`, etc.).

## Python and Development Tools

Make sure you have Python 3.8 or higher installed:
```bash
python3 --version
```

Install pip if not already installed:
```bash
sudo apt install python3-pip
```

## Workspace Setup

Create a ROS 2 workspace for your projects:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Simulation Environment (Gazebo)

Install Gazebo Harmonic for simulation:
```bash
sudo apt install ros-humble-gazebo-*
```

## Troubleshooting

### Common Issues

1. **Command 'ros2' not found**: Make sure you've sourced the setup.bash file in your current terminal session.

2. **Python import errors**: Ensure you're using Python 3 and that the ROS 2 Python packages are installed.

3. **Permission errors**: Don't run ROS 2 commands with sudo unless specifically instructed.

### Getting Help

- Check the official ROS 2 documentation: https://docs.ros.org/en/humble/
- Ask questions on ROS Answers: https://answers.ros.org/
- Join the ROS Discourse forum: https://discourse.ros.org/