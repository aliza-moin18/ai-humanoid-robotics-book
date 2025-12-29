---
sidebar_position: 4
---

# Chapter 11.4: Lab: Sim-to-Real Transfer

## Lab Overview

This lab provides hands-on experience with sim-to-real transfer techniques, applying the concepts learned in previous chapters to transfer a model trained in Isaac Sim to a real humanoid robot. You will implement domain randomization techniques, optimize your model for edge deployment, and evaluate the transfer performance.

### Learning Objectives

By the end of this lab, you will be able to:
1. Implement domain randomization in Isaac Sim for robust model training
2. Optimize a trained model for deployment on edge hardware
3. Evaluate sim-to-real transfer performance
4. Identify and address reality gap issues

### Prerequisites

- Completed Chapters 11.1-11.3
- Access to Isaac Sim environment
- Access to a humanoid robot platform (physical or simulated)
- Basic understanding of ROS 2 and Isaac ROS

## Lab Setup

### Environment Requirements

1. **Simulation Environment**: Isaac Sim 2023.1.1 with humanoid robot model
2. **Development Environment**: Ubuntu 22.04 with ROS 2 Humble
3. **Edge Hardware**: NVIDIA Jetson AGX Orin or equivalent
4. **Robot Platform**: Compatible humanoid robot with Isaac ROS support

### Initial Configuration

1. Launch Isaac Sim with your humanoid robot model
2. Verify Isaac ROS bridge connectivity
3. Ensure all sensors and actuators are properly configured

## Exercise 1: Implement Domain Randomization

### Task 1.1: Visual Domain Randomization

Create a domain randomization script that randomizes visual properties during training:

```python
# domain_randomization.py
import omni
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.materials import OmniPBR
import random

class VisualDomainRandomizer:
    def __init__(self, robot_prim_path, environment_prim_path):
        self.robot_prim_path = robot_prim_path
        self.environment_prim_path = environment_prim_path
        self.materials = self._get_all_materials()
        
    def _get_all_materials(self):
        # Get all materials in robot and environment
        materials = []
        # Implementation to collect materials
        return materials
    
    def randomize_visual_properties(self):
        """Randomize visual properties including colors, textures, and lighting"""
        for material in self.materials:
            # Randomize roughness (0.1 to 0.9)
            material.set_roughness(random.uniform(0.1, 0.9))
            
            # Randomize metallic value (0.0 to 1.0)
            material.set_metallic(random.uniform(0.0, 1.0))
            
            # Randomize color
            color = [random.uniform(0.2, 1.0) for _ in range(3)]
            material.set_color(color)
        
        # Randomize lighting conditions
        self._randomize_lighting()
    
    def _randomize_lighting(self):
        """Randomize lighting in the environment"""
        stage = omni.usd.get_context().get_stage()
        
        # Find all lights in the scene
        lights = [prim for prim in stage.TraverseAll() if prim.IsA(UsdLux.LightBase)]
        
        for light in lights:
            # Randomize intensity
            light.GetAttribute("inputs:intensity").Set(random.uniform(100, 1000))
            
            # Randomize color temperature
            light.GetAttribute("inputs:color").Set(
                carb.Float3(random.uniform(0.5, 1.0), 
                           random.uniform(0.5, 1.0), 
                           random.uniform(0.5, 1.0))
            )
```

### Task 1.2: Physical Domain Randomization

Implement physical property randomization:

```python
# physical_randomization.py
from omni.isaac.core.articulations import ArticulationView
import random

class PhysicalDomainRandomizer:
    def __init__(self, robot_articulation_view):
        self.robot = robot_articulation_view
    
    def randomize_physics_properties(self):
        """Randomize physical properties of the robot"""
        # Randomize link masses (±20% variation)
        for i in range(self.robot.num_bodies):
            current_mass = self.robot.get_mass_matrix()[i, i, 0]
            mass_variation = random.uniform(0.8, 1.2)
            new_mass = current_mass * mass_variation
            # Apply new mass to the link
            
        # Randomize joint friction
        for i in range(self.robot.num_dof):
            friction_range = (0.0, 0.5)
            random_friction = random.uniform(*friction_range)
            self.robot.set_joint_friction(i, random_friction)
        
        # Randomize joint damping
        for i in range(self.robot.num_dof):
            damping_range = (0.01, 0.1)
            random_damping = random.uniform(*damping_range)
            self.robot.set_joint_damping(i, random_damping)
```

## Exercise 2: Train Model with Domain Randomization

### Task 2.1: Training Loop with Randomization

Implement a training loop that applies domain randomization at specified intervals:

```python
# training_with_randomization.py
import torch
import torch.nn as nn
import numpy as np

class TrainingWithRandomization:
    def __init__(self, model, robot_env, visual_randomizer, physical_randomizer):
        self.model = model
        self.robot_env = robot_env
        self.visual_randomizer = visual_randomizer
        self.physical_randomizer = physical_randomizer
        self.optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
        self.criterion = nn.MSELoss()
        
    def train_step(self, num_episodes=1000, randomization_interval=10):
        """Training loop with domain randomization"""
        for episode in range(num_episodes):
            # Apply domain randomization at intervals
            if episode % randomization_interval == 0:
                self.visual_randomizer.randomize_visual_properties()
                self.physical_randomizer.randomize_physics_properties()
                print(f"Applied domain randomization at episode {episode}")
            
            # Reset environment
            obs = self.robot_env.reset()
            total_reward = 0
            
            # Episode execution
            for step in range(1000):  # Max steps per episode
                # Get action from model
                action = self.model(torch.tensor(obs).float())
                
                # Execute action in environment
                next_obs, reward, done, info = self.robot_env.step(action.detach().numpy())
                
                # Compute loss and update model
                target = torch.tensor(next_obs).float()
                loss = self.criterion(self.model(torch.tensor(obs).float()), target)
                
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
                
                obs = next_obs
                total_reward += reward
                
                if done:
                    break
            
            # Log episode results
            if episode % 100 == 0:
                print(f"Episode {episode}, Total Reward: {total_reward}")
```

### Task 2.2: Progressive Randomization

Implement progressive randomization that increases the randomization range over time:

```python
def progressive_randomization(episode, total_episodes):
    """Calculate randomization intensity based on training progress"""
    progress = episode / total_episodes
    # Increase randomization intensity gradually, max at 80% progress
    intensity = min(progress * 1.25, 1.0)  # Cap at 1.0
    
    # Define base ranges for randomization
    base_roughness_range = (0.1, 0.9)
    base_mass_variation = (0.8, 1.2)
    
    # Apply intensity to ranges
    roughness_range = (
        0.1 + (base_roughness_range[0] - 0.1) * (1 - intensity),
        0.9 - (0.9 - base_roughness_range[1]) * (1 - intensity)
    )
    
    mass_variation = (
        1.0 - (1.0 - base_mass_variation[0]) * intensity,
        1.0 + (base_mass_variation[1] - 1.0) * intensity
    )
    
    return roughness_range, mass_variation
```

## Exercise 3: Model Optimization for Edge Deployment

### Task 3.1: Convert Model to TensorRT

Optimize your trained model for deployment on Jetson hardware:

```python
# model_optimization.py
import torch
import torch_tensorrt

def optimize_model_for_jetson(model, sample_input):
    """Optimize PyTorch model for NVIDIA Jetson using Torch-TensorRT"""
    
    # Trace the model
    traced_model = torch.jit.trace(model.eval(), sample_input)
    
    # Compile with Torch-TensorRT
    optimized_model = torch_tensorrt.compile(
        traced_model,
        inputs=[torch_tensorrt.Input(
            min_shape=[1, *sample_input.shape[1:]],
            opt_shape=[8, *sample_input.shape[1:]],
            max_shape=[16, *sample_input.shape[1:]]
        )],
        enabled_precisions={torch.float, torch.half},  # Use FP32 and FP16
        workspace_size=1 << 28,  # 256 MiB
        debug=True
    )
    
    return optimized_model

def save_optimized_model(optimized_model, save_path):
    """Save the optimized model"""
    torch.jit.save(optimized_model, save_path)
    print(f"Optimized model saved to {save_path}")
```

### Task 3.2: Create Deployment Package

Package the optimized model with necessary configuration files:

```python
# create_deployment_package.py
import os
import shutil
import json
import yaml

def create_deployment_package(model_path, config_path, output_dir):
    """Create a deployment package for edge hardware"""
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Copy optimized model
    model_filename = os.path.basename(model_path)
    shutil.copy(model_path, os.path.join(output_dir, model_filename))
    
    # Copy configuration
    config_filename = os.path.basename(config_path)
    shutil.copy(config_path, os.path.join(output_dir, config_filename))
    
    # Create deployment manifest
    manifest = {
        "model_file": model_filename,
        "config_file": config_filename,
        "deployment_timestamp": "2024-01-01T00:00:00Z",
        "hardware_target": "jetson-agx-orin",
        "dependencies": [
            "isaac_ros_common",
            "isaac_ros_visual_slam",
            "isaac_ros_detect_net"
        ]
    }
    
    # Save manifest
    with open(os.path.join(output_dir, "manifest.json"), 'w') as f:
        json.dump(manifest, f, indent=2)
    
    # Create README with deployment instructions
    readme_content = f"""
# Deployment Package for Humanoid Robot AI

This package contains:
- Model: {model_filename}
- Configuration: {config_filename}

## Deployment Instructions

1. Transfer this package to your Jetson device
2. Run the deployment script: `./deploy.sh`
3. Verify the deployment with: `./verify.sh`

For troubleshooting, see the documentation.
"""
    
    with open(os.path.join(output_dir, "README.md"), 'w') as f:
        f.write(readme_content)
    
    print(f"Deployment package created at {output_dir}")

# Example usage
if __name__ == "__main__":
    create_deployment_package(
        "optimized_model.ts",
        "robot_config.yaml", 
        "deployment_package"
    )
```

## Exercise 4: Deploy and Evaluate

### Task 4.1: Deploy Model to Robot

Deploy the optimized model to your robot platform:

```bash
#!/bin/bash
# deploy.sh - Deployment script for Jetson device

set -e  # Exit on any error

# Configuration
ROBOT_IP="192.168.1.100"  # Replace with actual robot IP
PACKAGE_PATH="./deployment_package"
REMOTE_PATH="/home/jetson/robot_models"

# Transfer package to robot
echo "Transferring package to robot..."
scp -r $PACKAGE_PATH jetson@$ROBOT_IP:$REMOTE_PATH

# SSH into robot and install package
ssh jetson@$ROBOT_IP << 'EOF'
cd /home/jetson
cd robot_models/deployment_package

# Install dependencies if needed
# pip3 install -r requirements.txt

# Move model to appropriate location
sudo cp optimized_model.ts /opt/robot/models/
sudo cp robot_config.yaml /opt/robot/config/

# Restart robot services
sudo systemctl restart robot-ai.service
sudo systemctl restart isaac-ros-bridge.service

echo "Deployment completed successfully"
EOF

echo "Model deployed to robot at $ROBOT_IP"
```

### Task 4.2: Evaluate Transfer Performance

Evaluate how well your model performs in the real world:

```python
# evaluation.py
import numpy as np
import matplotlib.pyplot as plt

class TransferEvaluator:
    def __init__(self, sim_model, real_robot_interface):
        self.sim_model = sim_model
        self.real_robot = real_robot_interface
    
    def evaluate_performance(self, test_scenarios):
        """Evaluate model performance in simulation vs reality"""
        sim_results = []
        real_results = []
        
        for scenario in test_scenarios:
            # Test in simulation
            sim_reward = self._test_in_simulation(scenario)
            sim_results.append(sim_reward)
            
            # Test on real robot
            real_reward = self._test_on_real_robot(scenario)
            real_results.append(real_reward)
        
        # Calculate transfer metrics
        sim_mean = np.mean(sim_results)
        real_mean = np.mean(real_results)
        performance_gap = sim_mean - real_mean
        relative_gap = performance_gap / sim_mean if sim_mean != 0 else 0
        
        print(f"Simulation Performance: {sim_mean:.3f}")
        print(f"Real Robot Performance: {real_mean:.3f}")
        print(f"Performance Gap: {performance_gap:.3f}")
        print(f"Relative Gap: {relative_gap:.3f}")
        
        # Create comparison plot
        self._plot_comparison(sim_results, real_results)
        
        return {
            'sim_mean': sim_mean,
            'real_mean': real_mean,
            'performance_gap': performance_gap,
            'relative_gap': relative_gap
        }
    
    def _test_in_simulation(self, scenario):
        """Test model in simulation for given scenario"""
        # Reset simulation to scenario
        obs = self.sim_model.reset_scenario(scenario)
        total_reward = 0
        
        for step in range(1000):  # Max steps
            action = self.sim_model.get_action(obs)
            obs, reward, done, info = self.sim_model.step(action)
            total_reward += reward
            
            if done:
                break
        
        return total_reward
    
    def _test_on_real_robot(self, scenario):
        """Test model on real robot for given scenario"""
        # Set up real robot for scenario
        self.real_robot.setup_scenario(scenario)
        obs = self.real_robot.get_observation()
        total_reward = 0
        
        for step in range(1000):  # Max steps
            action = self.sim_model.get_action(obs)
            obs, reward, done, info = self.real_robot.execute_action(action)
            total_reward += reward
            
            if done:
                break
        
        return total_reward
    
    def _plot_comparison(self, sim_results, real_results):
        """Plot comparison between simulation and real performance"""
        fig, ax = plt.subplots(figsize=(10, 6))
        
        x = np.arange(len(sim_results))
        width = 0.35
        
        ax.bar(x - width/2, sim_results, width, label='Simulation', alpha=0.8)
        ax.bar(x + width/2, real_results, width, label='Reality', alpha=0.8)
        
        ax.set_xlabel('Test Scenario')
        ax.set_ylabel('Performance')
        ax.set_title('Simulation vs Reality Performance Comparison')
        ax.legend()
        
        plt.tight_layout()
        plt.savefig('transfer_evaluation.png')
        plt.show()
```

## Exercise 5: Analysis and Improvement

### Task 5.1: Identify Reality Gap Issues

Analyze where your model is underperforming and identify potential causes:

```python
def analyze_reality_gap(evaluator_results, test_scenarios):
    """Analyze where the reality gap is most significant"""
    
    significant_gaps = []
    
    for i, scenario in enumerate(test_scenarios):
        sim_result = evaluator_results['sim_results'][i]
        real_result = evaluator_results['real_results'][i]
        gap = sim_result - real_result
        
        if gap > evaluator_results['performance_threshold']:
            significant_gaps.append({
                'scenario': scenario,
                'sim_result': sim_result,
                'real_result': real_result,
                'gap': gap,
                'relative_gap': gap / sim_result if sim_result != 0 else 0
            })
    
    # Sort by gap size
    significant_gaps.sort(key=lambda x: x['gap'], reverse=True)
    
    print("Top scenarios with significant reality gaps:")
    for gap_info in significant_gaps[:5]:
        print(f"Scenario: {gap_info['scenario']}")
        print(f"  Simulation: {gap_info['sim_result']:.3f}")
        print(f"  Reality: {gap_info['real_result']:.3f}")
        print(f"  Gap: {gap_info['gap']:.3f}")
        print(f"  Relative Gap: {gap_info['relative_gap']:.3f}")
        print()
    
    return significant_gaps
```

### Task 5.2: Propose Improvements

Based on your analysis, propose improvements to reduce the reality gap:

```python
def propose_improvements(significant_gaps):
    """Propose improvements based on reality gap analysis"""
    
    improvements = {
        'visual_domain': [],
        'physical_domain': [],
        'sensor_domain': [],
        'control_domain': []
    }
    
    for gap_info in significant_gaps:
        scenario = gap_info['scenario']
        
        # Analyze what aspects of the scenario might be causing the gap
        if 'lighting' in scenario or 'visual' in scenario:
            improvements['visual_domain'].append({
                'issue': f"Large gap in {scenario} scenario",
                'suggestion': "Increase visual domain randomization range for lighting conditions"
            })
        
        if 'dynamics' in scenario or 'movement' in scenario:
            improvements['physical_domain'].append({
                'issue': f"Large gap in {scenario} scenario", 
                'suggestion': "Include more physical parameter variations in training"
            })
        
        if 'sensor' in scenario or 'perception' in scenario:
            improvements['sensor_domain'].append({
                'issue': f"Large gap in {scenario} scenario",
                'suggestion': "Add sensor noise and delay randomization"
            })
        
        if 'control' in scenario or 'precision' in scenario:
            improvements['control_domain'].append({
                'issue': f"Large gap in {scenario} scenario",
                'suggestion': "Implement control delay and imprecision randomization"
            })
    
    # Print improvement suggestions
    for domain, suggestions in improvements.items():
        if suggestions:
            print(f"\n{domain.upper()} IMPROVEMENTS:")
            for suggestion in suggestions:
                print(f"  - {suggestion['suggestion']} (Issue: {suggestion['issue']})")
    
    return improvements
```

## Lab Report Requirements

Document your findings in a comprehensive lab report including:

1. **Methodology**: Describe your domain randomization approach
2. **Results**: Present performance metrics for simulation vs reality
3. **Analysis**: Discuss the identified reality gaps and their causes
4. **Improvements**: Propose specific improvements for future iterations
5. **Conclusion**: Summarize the effectiveness of your sim-to-real transfer approach

## Assessment Rubric

Your lab will be assessed based on:

- **Implementation (40%)**: Correct implementation of domain randomization and model optimization
- **Evaluation (30%)**: Thorough evaluation of sim-to-real transfer performance
- **Analysis (20%)**: Quality of reality gap analysis and improvement proposals
- **Documentation (10%)**: Clarity and completeness of lab report

## Troubleshooting Tips

1. **Model Performance Degrades**: Check if randomization ranges are too wide
2. **Deployment Issues**: Verify JetPack version compatibility
3. **Performance Mismatch**: Consider additional domain randomization factors
4. **Hardware Limitations**: Optimize model size and complexity for target hardware

## Next Steps

After completing this lab, you should:
1. Implement the suggested improvements to your domain randomization
2. Retrain your model with enhanced randomization
3. Perform another sim-to-real transfer evaluation
4. Compare results to measure improvement in transfer performance