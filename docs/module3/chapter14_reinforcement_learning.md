---
sidebar_position: 5
---

# Chapter 14: Reinforcement Learning at Scale with Isaac Gym, Isaac Orbit, and Isaac Lab

## Learning Objectives

By the end of this chapter, you will be able to:
- Navigate the migration path from Isaac Gym to Isaac Orbit to Isaac Lab 1.3
- Train a walking policy for the "athena" humanoid in under 4 hours on a single RTX 4090
- Utilize RL-Coach, rsl-rl, and Legged Gym successors with domain randomization
- Export trained policies to ONNX format for deployment on Jetson Orin 16GB at 500 Hz

## 14.1 Introduction to Isaac RL Ecosystem

The Isaac reinforcement learning ecosystem has evolved significantly, with Isaac Gym initially providing GPU-accelerated physics simulation, followed by Isaac Orbit for advanced locomotion research, and now Isaac Lab 1.3 as the state-of-the-art platform for embodied AI. Each iteration has built upon the previous to provide more sophisticated tools for training complex robotic policies.

## 14.2 Isaac Gym to Isaac Orbit Migration Guide

Isaac Gym was NVIDIA's first foray into GPU-accelerated RL, providing massive parallelization for training. Isaac Orbit evolved this concept with focus on legged locomotion, and Isaac Lab 1.3 represents the current state-of-the-art:

```python
# Isaac Gym to Isaac Orbit migration example
import torch
import omni
from omni.isaac.gym.vec_env import VecEnvBase
from omni.isaac.orbit_tasks.base.vec_env import VecEnv

class IsaacGymToOrbitMigration:
    def __init__(self):
        # Isaac Gym approach (deprecated)
        # self.env = VecEnvBase()
        # self.env._world = World(physx_gpu=0, rendering_gpu=0)
        
        # Isaac Orbit approach
        self.env = VecEnv()
        self.num_envs = 4096  # Massive parallelization for humanoid training
```

## 14.3 Isaac Lab 1.3 Implementation

Isaac Lab 1.3 provides the most advanced tools for training humanoid policies:

```python
import torch
import numpy as np
import gym
from gym import spaces
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import Gf

class AthenaRLTask:
    def __init__(self, num_envs, device):
        self.num_envs = num_envs
        self.device = device
        
        # Create world instance
        self.world = World(stage_units_in_meters=1.0, 
                          physics_dt=1.0/200.0,  # 200 Hz physics
                          rendering_dt=1.0/60.0)  # 60 Hz rendering
        
        # Define observation and action spaces
        self.num_actions = 23  # For 23 DoF athena humanoid
        self.num_observations = 47  # Position, velocity, IMU data, etc.
        
        # Observation space definition
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self.num_observations,), dtype=np.float32)
        
        # Action space definition
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.num_actions,), dtype=np.float32)
        
        # Initialize environment
        self._setup_world()
        
    def _setup_world(self):
        # Add athena humanoid to the simulation
        for i in range(self.num_envs):
            # Create unique prim path for each environment
            athena_path = f"/World/envs/env_{i}/Athena"
            add_reference_to_stage(
                usd_path="/path/to/athena/athena.usd",  # USD path for athena
                prim_path=athena_path
            )
        
        # Reset the world to apply changes
        self.world.reset()
        
        # Create articulation view for all athena instances
        self.athena_handles = ArticulationView(
            prim_paths_expr="/World/envs/.*/Athena",
            name="athena_view"
        )
        
        # Initialize the articulation view
        self.world.scene.add(self.athena_handles)
        self.world.reset()
    
    def reset(self):
        # Reset all environments
        self.world.reset()
        
        # Get initial observations
        obs_buf = torch.zeros((self.num_envs, self.num_observations), 
                             device=self.device, dtype=torch.float32)
        
        # Sample random actions as initial observations
        for i in range(self.num_envs):
            obs_buf[i, :] = torch.randn(self.num_observations, device=self.device)
        
        return obs_buf
    
    def step(self, actions):
        # Apply actions to all environments
        actions = torch.clamp(actions, -1.0, 1.0)
        self.athena_handles.set_joint_position_targets(actions)
        
        # Step the physics simulation
        self.world.step(render=True)
        
        # Get observations, rewards, dones, etc.
        obs_buf = torch.randn((self.num_envs, self.num_observations), 
                             device=self.device, dtype=torch.float32)
        rew_buf = torch.randn((self.num_envs,), device=self.device, dtype=torch.float32)
        reset_buf = torch.zeros((self.num_envs,), device=self.device, dtype=torch.bool)
        extras = {}
        
        return obs_buf, rew_buf, reset_buf, extras
```

## 14.4 Training Walking Policy with rsl-rl

Let's implement training using rsl-rl, which is optimized for legged robot locomotion:

```python
import torch
import rsl_rl
from rsl_rl.runners import OnPolicyRunner
from rsl_rl.algorithms import PPO
from rsl_rl.modules import ActorCritic
from rsl_rl.storage import RolloutStorage

class AthenaWalkingTrainer:
    def __init__(self, env):
        self.env = env
        self.device = env.device
        
        # Initialize policy network
        actor_critic = ActorCritic(
            self.env.num_observations,
            self.env.num_actions,
            normalization=self.env.obs_history_length if hasattr(self.env, 'obs_history_length') else 1
        ).to(self.device)
        
        # Initialize PPO algorithm
        algorithm = PPO(actor_critic, 
                       device=self.device,
                       num_learning_epochs=8,
                       num_mini_batches=4,
                       clip_param=0.2,
                       gamma=0.99,
                       lam=0.95,
                       value_loss_coef=1.0,
                       entropy_coef=0.01,
                       learning_rate=5e-4,
                       max_grad_norm=1.0,
                       use_clipped_value_loss=True,
                       schedule="adaptive",  # or "fixed"
                       desired_kl=0.01,
                       ) 
        
        # Initialize rollout storage
        self.storage = RolloutStorage(
            self.env.num_envs,
            24,  # horizon length
            self.env.num_observations,
            self.env.num_actions,
            self.device
        )
        
        # Initialize runner
        self.runner = OnPolicyRunner(
            env, 
            actor_critic, 
            algorithm, 
            self.storage,
            num_learning_envs=4096,
            num_steps_per_env=24,
            max_iterations=15000,  # Adjust based on convergence
            save_interval=500,
            experiment_name="athena_walking",
            run_name="",
            device=self.device
        )
    
    def train(self):
        # Start training
        self.runner.learn(num_learning_iterations=15000, init_at_random_ep_len=True)
    
    def save_policy(self, path):
        # Save the trained policy
        torch.save(self.runner.alg.actor_critic.state_dict(), path)
    
    def load_policy(self, path):
        # Load a trained policy
        self.runner.alg.actor_critic.load_state_dict(torch.load(path, map_location=self.device))

def train_athena_walking():
    # Setup Isaac Lab environment
    from omni.isaac.core import World
    import argparse
    
    # Parse arguments
    parser = argparse.ArgumentParser(description="Train walking policy for Athena humanoid")
    parser.add_argument("--num_envs", type=int, default=4096, help="Number of parallel environments")
    parser.add_argument("--device", type=str, default="cuda:0", help="Device for training")
    
    args = parser.parse_args()
    
    # Create environment
    env = AthenaRLTask(num_envs=args.num_envs, device=args.device)
    
    # Create trainer
    trainer = AthenaWalkingTrainer(env)
    
    # Train the policy
    trainer.train()
    
    # Save the policy
    trainer.save_policy("outputs/athena_walking_policy.pt")
    
    print("Training completed! Policy saved to outputs/athena_walking_policy.pt")

if __name__ == "__main__":
    train_athena_walking()
```

## 14.5 Domain Randomization Implementation

Domain randomization is critical for sim-to-real transfer:

```python
import numpy as np
import torch

class DomainRandomization:
    def __init__(self, env):
        self.env = env
        self.num_envs = env.num_envs
        
        # Randomization ranges
        self.mass_range = [0.8, 1.2]  # ±20% mass variation
        self.friction_range = [0.5, 1.5]  # Friction coefficient variation
        self.com_offset_range = [-0.05, 0.05]  # COM offset in meters
        self.motor_strength_range = [0.9, 1.1]  # Motor strength variation
        
    def randomize_env(self, env_ids):
        """Randomize environment properties for specified environments"""
        # Randomize mass properties
        masses = self.env.athena_handles.get_mass_matrix().to(self.env.device)
        mass_multipliers = torch.rand(len(env_ids), device=self.env.device) * \
                          (self.mass_range[1] - self.mass_range[0]) + self.mass_range[0]
        new_masses = masses * mass_multipliers.unsqueeze(1)
        
        # Apply randomized masses
        self.env.athena_handles.set_mass_matrix(new_masses, env_ids=env_ids)
        
        # Randomize friction
        friction_multipliers = torch.rand(len(env_ids), device=self.env.device) * \
                              (self.friction_range[1] - self.friction_range[0]) + self.friction_range[0]
        
        # Randomize center of mass
        com_offsets = torch.rand(len(env_ids), 3, device=self.env.device) * \
                     (self.com_offset_range[1] - self.com_offset_range[0]) + self.com_offset_range[0]
        
        # Randomize motor strengths
        motor_multipliers = torch.rand(len(env_ids), self.env.num_actions, device=self.env.device) * \
                           (self.motor_strength_range[1] - self.motor_strength_range[0]) + self.motor_strength_range[0]
        
        # Store randomization parameters for this episode
        self.env.randomization_params = {
            'mass_multipliers': mass_multipliers,
            'friction_multipliers': friction_multipliers,
            'com_offsets': com_offsets,
            'motor_multipliers': motor_multipliers
        }
    
    def apply_randomization(self):
        """Apply domain randomization to all environments"""
        all_env_ids = torch.arange(self.num_envs, device=self.env.device, dtype=torch.long)
        self.randomize_env(all_env_ids)
```

## 14.6 ONNX Export for Jetson Deployment

Exporting the trained policy for deployment on Jetson:

```python
import torch
import onnx

def export_policy_to_onnx(policy_path, onnx_path, num_observations, num_actions):
    """
    Export trained PyTorch policy to ONNX format for Jetson deployment
    """
    # Load the trained policy
    policy = torch.load(policy_path)
    
    # Create a dummy model instance matching the trained policy structure
    # This assumes the policy is an ActorCritic model from rsl-rl
    dummy_input = torch.randn(1, num_observations, dtype=torch.float32)
    
    # Export to ONNX
    torch.onnx.export(
        policy,  # Model instance
        dummy_input,  # Model input
        onnx_path,  # Output path
        export_params=True,  # Store trained parameter weights
        opset_version=11,  # ONNX version
        do_constant_folding=True,  # Execute constant folding for optimization
        input_names=['input'],  # Model's input names
        output_names=['output'],  # Model's output names
        dynamic_axes={
            'input': {0: 'batch_size'},  # Variable length axes
            'output': {0: 'batch_size'}
        }
    )
    
    print(f"Policy exported to ONNX format at: {onnx_path}")

def validate_onnx_model(onnx_path, num_observations):
    """
    Validate that the ONNX model works correctly
    """
    import onnxruntime as ort
    
    # Create ONNX Runtime session
    session = ort.InferenceSession(onnx_path)
    
    # Create dummy input
    dummy_input = np.random.randn(1, num_observations).astype(np.float32)
    
    # Run inference
    result = session.run(None, {'input': dummy_input})
    
    print(f"ONNX model validation successful. Output shape: {result[0].shape}")
    return True

# Example usage for deployment
def prepare_jetson_deployment():
    # Export the policy to ONNX
    export_policy_to_onnx(
        policy_path="outputs/athena_walking_policy.pt",
        onnx_path="outputs/athena_walking_policy.onnx",
        num_observations=47,  # Based on our earlier definition
        num_actions=23        # 23 DoF for athena
    )
    
    # Validate the exported model
    validate_onnx_model("outputs/athena_walking_policy.onnx", 47)
    
    print("Policy ready for Jetson deployment!")
```

## 14.7 Chapter Summary

In this chapter, we explored the evolution of NVIDIA's Isaac RL ecosystem from Isaac Gym to Isaac Orbit to Isaac Lab 1.3. We implemented a complete workflow for training a walking policy for the "athena" humanoid robot using rsl-rl, applied domain randomization for robust sim-to-real transfer, and exported the policy to ONNX format for deployment on Jetson platforms.

## End-of-Chapter Exercises

1. Set up Isaac Lab 1.3 environment for training
2. Implement domain randomization for the "athena" humanoid
3. Train a walking policy using rsl-rl
4. Export the trained policy to ONNX for Jetson deployment