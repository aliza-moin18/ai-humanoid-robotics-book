---
sidebar_position: 2
---

# Section 1.2: Unity Installation and Configuration

## Overview

This section provides detailed instructions for installing and configuring the Unity visualization platform. Students will learn to set up Unity with appropriate packages for robotics visualization and verify the installation.

## System Requirements

Before installing Unity, ensure your system meets the following requirements:
- Operating System: Windows 10/11, macOS 10.14+, or Ubuntu 20.04+
- RAM: 8GB minimum, 16GB recommended
- Storage: 20GB available space for Unity Hub + 15GB for Unity Editor
- Graphics: DirectX 10, DirectX 11, or OpenGL 4.3+ compatible GPU
- CPU: SSE2 instruction set support

## Installation Steps

### Step 1: Install Unity Hub

1. Download Unity Hub from the [official Unity website](https://unity.com/download)
2. Run the installer and follow the on-screen instructions
3. Sign in with your Unity ID or create a new account

### Step 2: Install Unity Editor

1. Open Unity Hub
2. Go to the "Installs" tab
3. Click "Add" to install a new Unity version
4. Select the latest Long Term Support (LTS) version
5. In the installer, select the following modules:
   - Windows Build Support (if on Windows)
   - Linux Build Support (if on Linux)
   - Visual Studio Editor (or your preferred IDE)
   - Documentation

### Step 3: Install Robotics Packages

1. Open Unity Hub and create a new 3D project
2. In the Unity Editor, open the Package Manager (Window > Package Manager)
3. Install the following packages:
   - **Unity Robotics Hub**: Contains tools for robotics simulation
   - **Universal Render Pipeline (URP)**: For optimized rendering
   - **XR Interaction Toolkit**: For VR/AR capabilities (optional)

## Configuration

### Unity Robotics Setup

1. Import the Unity Robotics Package:
   - Go to Assets > Import Package > Custom Package
   - Import the Unity Robotics Hub package

2. Set up the physics environment:
   - Go to Edit > Project Settings > Physics
   - Adjust the following parameters:
     - Gravity: (0, -9.81, 0)
     - Default Material: Create a material with appropriate friction values

3. Configure rendering for robotics visualization:
   - Create a new URP Asset (Assets > Create > Rendering > Universal Render Pipeline > Pipeline Asset)
   - Apply this asset in Project Settings > Graphics

## Verification

Test the installation by creating a simple scene:

1. Create a new 3D project in Unity Hub
2. Add a cube to the scene (GameObject > 3D Object > Cube)
3. Add a physics material to the cube
4. Play the scene to verify physics simulation works

## Troubleshooting

- If Unity fails to install, ensure .NET Framework 4.8+ is installed on Windows
- For graphics issues, update your graphics drivers
- If packages fail to install, verify your internet connection and Unity license

## Next Steps

After successfully installing and configuring Unity, proceed to [Section 1.3: Integration between Gazebo and Unity](../chapter-1/section-1-3-integration).