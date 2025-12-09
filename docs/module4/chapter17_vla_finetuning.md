---
sidebar_position: 3
---

# Chapter 17: Building and Fine-tuning Your Own Vision-Language-Action Model

## Learning Objectives

By the end of this chapter, you will be able to:
- Fine-tune OpenVLA-7B on your Isaac Sim dataset for the "athena" humanoid
- Implement LoRA and QLoRA techniques for efficient fine-tuning on 1-4 RTX 4090 GPUs
- Generate high-quality VLA training data with action chunking and language annotation
- Optimize your model for deployment at 8-12 tokens/sec on Jetson Orin 16GB with 4-bit quantization

## 17.1 Introduction to VLA Fine-Tuning

Fine-tuning Vision-Language-Action models requires specialized techniques that differ significantly from traditional computer vision or NLP fine-tuning. The key challenges include:

- **Multi-modal alignment**: Coordinating visual, textual, and action spaces
- **Temporal consistency**: Ensuring smooth action trajectories over time
- **Robotic specificity**: Training models to understand robot embodiment and physics
- **Safety constraints**: Ensuring models output physically realizable actions

This chapter walks you through the complete process of adapting OpenVLA-7B or other VLA models to the specific dynamics and morphology of your "athena" humanoid robot using your Isaac Sim dataset.

## 17.2 Setting Up the Fine-Tuning Environment

Before we begin fine-tuning, we need to set up the appropriate environment with all necessary dependencies:

```bash
# requirements_vla_finetune.txt
torch>=2.3.0
transformers>=4.36.0
peft>=0.6.0
bitsandbytes>=0.41.0
accelerate>=0.24.0
datasets>=2.14.0
trl>=0.7.4
numpy>=1.24.3
opencv-python>=4.8.0.74
matplotlib>=3.7.2
tqdm>=4.65.0
wandb>=0.15.12  # For experiment tracking
sentence-transformers>=2.5.0
```

Now let's implement the complete fine-tuning pipeline:

```python
import torch
import numpy as np
from datasets import Dataset
from transformers import AutoProcessor, AutoModelForVision2Seq, TrainingArguments
from peft import LoraConfig, get_peft_model, TaskType
from trl import SFTTrainer
from PIL import Image
import json
import os
from typing import List, Dict, Any
import pandas as pd

class VLADatasetPreparation:
    """
    Prepare and process Isaac Sim dataset for VLA fine-tuning
    """
    def __init__(self, isaac_data_path: str, output_path: str):
        self.isaac_data_path = isaac_data_path
        self.output_path = output_path
        self.processor = None  # Will be set during processing

    def load_isaac_episodes(self) -> List[Dict[str, Any]]:
        """
        Load Isaac Sim recordings and convert to VLA training format
        """
        episodes = []

        # Iterate through all episode directories
        for episode_dir in os.listdir(self.isaac_data_path):
            episode_path = os.path.join(self.isaac_data_path, episode_dir)
            if not os.path.isdir(episode_path):
                continue

            # Load episode metadata
            try:
                with open(os.path.join(episode_path, 'metadata.json'), 'r') as f:
                    metadata = json.load(f)

                # Load trajectory data
                traj_path = os.path.join(episode_path, 'trajectory.npz')
                if os.path.exists(traj_path):
                    traj_data = np.load(traj_path)

                    episode = {
                        'episode_id': episode_dir,
                        'images': self.load_images_from_episode(episode_path),
                        'actions': traj_data['actions'],
                        'observations': traj_data.get('observations', []),
                        'language_instructions': metadata.get('instructions', []),
                        'task_descriptions': metadata.get('task_descriptions', []),
                        'episode_length': len(traj_data['actions']),
                        'success': metadata.get('success', False)
                    }

                    episodes.append(episode)

            except Exception as e:
                print(f"Error loading episode {episode_dir}: {e}")
                continue

        return episodes

    def load_images_from_episode(self, episode_path: str) -> List[str]:
        """
        Get list of image file paths for an episode
        """
        images_path = os.path.join(episode_path, 'images')
        if not os.path.exists(images_path):
            return []

        image_files = sorted([
            os.path.join(images_path, f) for f in os.listdir(images_path)
            if f.endswith(('.png', '.jpg', '.jpeg'))
        ])

        return image_files

    def create_vla_training_data(self, episodes: List[Dict[str, Any]], max_length: int = 512) -> Dataset:
        """
        Convert episodes to VLA training format
        """
        training_examples = []

        for episode in episodes:
            # Create training examples from episode
            for i in range(len(episode['images'])):
                if i >= len(episode['actions']):
                    continue

                # Get the current image
                image_path = episode['images'][i]

                # Create multiple training examples per image if multiple instructions exist
                instructions = episode['language_instructions']
                if not instructions:
                    # If no specific instructions, create generic ones
                    instructions = [f"perform the task"]

                for instruction in instructions:
                    # Format for VLA training
                    formatted_text = f"In: What action should the robot take to {instruction.lower()}?\nOut:"

                    # Add action as sequence of tokens
                    action = episode['actions'][i]

                    # Create training example
                    example = {
                        'image_path': image_path,
                        'text': formatted_text,
                        'action_sequence': action.tolist(),  # Convert to list for JSON serialization
                        'episode_id': episode['episode_id'],
                        'frame_index': i,
                        'success': episode['success']
                    }

                    training_examples.append(example)

        # Convert to HuggingFace dataset format
        df = pd.DataFrame(training_examples)
        dataset = Dataset.from_pandas(df)

        return dataset

    def tokenize_function(self, examples):
        """
        Tokenize examples for training
        """
        texts = examples['text']
        images = [Image.open(img_path) for img_path in examples['image_path']]

        # Process images and text with the processor
        inputs = self.processor(
            text=texts,
            images=images,
            return_tensors="pt",
            padding=True,
            truncation=True,
            max_length=512
        )

        # Add labels for training (same as input for generation tasks)
        inputs['labels'] = inputs['input_ids'].clone()

        return inputs

def prepare_fine_tuning_model(base_model_name: str, lora_config: Dict[str, Any]):
    """
    Prepare base VLA model with specified LoRA configuration for fine-tuning
    """
    # Load base model with quantization for memory efficiency
    model = AutoModelForVision2Seq.from_pretrained(
        base_model_name,
        torch_dtype=torch.float16,
        low_cpu_mem_usage=True,
        device_map="auto",  # Automatically distribute across available GPUs
        quantization_config=transformers.BitsAndBytesConfig(
            load_in_4bit=True,
            bnb_4bit_use_double_quant=True,
            bnb_4bit_quant_type="nf4",
            bnb_4bit_compute_dtype=torch.float16
        ) if base_model_name.startswith("openvla") else None
    )

    # Apply LoRA configuration
    lora_config = LoraConfig(
        task_type=TaskType.CAUSAL_LM,
        inference_mode=False,
        r=lora_config['r'],
        lora_alpha=lora_config['lora_alpha'],
        lora_dropout=lora_config['lora_dropout'],
        target_modules=lora_config['target_modules']
    )

    model = get_peft_model(model, lora_config)

    return model

def setup_accelerator():
    """
    Setup for multi-GPU training using HuggingFace Accelerate
    """
    from accelerate import Accelerator

    accelerator = Accelerator()

    # Print accelerator info
    print(f"Using {accelerator.num_processes} GPU(s)")
    if accelerator.is_main_process:
        print("Training on multiple GPUs enabled")

    return accelerator

# Training configuration
TRAINING_CONFIG = {
    'model_name': 'openvla/openvla-7b',
    'dataset_path': '/path/to/isaac_sim_athena_data',
    'output_dir': './fine_tuned_athena_vla/',
    'learning_rate': 5e-5,
    'batch_size': 1,  # Due to large model size
    'accumulation_steps': 16,  # Effective batch size = 1 * 16 = 16
    'num_epochs': 3,
    'warmup_steps': 100,
    'save_strategy': 'epoch',
    'logging_steps': 10,
    'evaluation_strategy': 'epoch',
    'gradient_checkpointing': True,  # Save memory during training
    'remove_unused_columns': False,
    'dataloader_pin_memory': True,
    'fp16': True,  # Mixed precision training
    'max_grad_norm': 1.0  # Gradient clipping
}

def run_vla_fine_tuning():
    """
    Execute complete VLA fine-tuning pipeline
    """
    print("🚀 Starting VLA Fine-tuning Pipeline for Athena Humanoid")

    # 1. Prepare dataset
    print("📊 Preparing Isaac Sim dataset...")
    dataset_prep = VLADatasetPreparation(
        isaac_data_path=TRAINING_CONFIG['dataset_path'],
        output_path=TRAINING_CONFIG['output_dir']
    )

    # Load episodes
    episodes = dataset_prep.load_isaac_episodes()
    print(f"Loaded {len(episodes)} episodes from Isaac Sim")

    # Create training dataset
    train_dataset = dataset_prep.create_vla_training_data(episodes)
    print(f"Created training dataset with {len(train_dataset)} examples")

    # 2. Initialize processor and model
    print("🔧 Initializing model and processor...")
    processor = AutoProcessor.from_pretrained(TRAINING_CONFIG['model_name'])
    dataset_prep.processor = processor  # Set processor for tokenization

    # Prepare model with LoRA
    lora_config = {
        'r': 64,
        'lora_alpha': 16,
        'lora_dropout': 0.1,
        'target_modules': [
            "q_proj", "k_proj", "v_proj", "o_proj",
            "gate_proj", "up_proj", "down_proj"
        ]
    }

    model = prepare_fine_tuning_model(TRAINING_CONFIG['model_name'], lora_config)

    # 3. Setup training arguments
    training_args = TrainingArguments(
        output_dir=TRAINING_CONFIG['output_dir'],
        num_train_epochs=TRAINING_CONFIG['num_epochs'],
        per_device_train_batch_size=TRAINING_CONFIG['batch_size'],
        gradient_accumulation_steps=TRAINING_CONFIG['accumulation_steps'],
        warmup_steps=TRAINING_CONFIG['warmup_steps'],
        learning_rate=TRAINING_CONFIG['learning_rate'],
        fp16=TRAINING_CONFIG['fp16'],
        logging_steps=TRAINING_CONFIG['logging_steps'],
        save_strategy=TRAINING_CONFIG['save_strategy'],
        evaluation_strategy=TRAINING_CONFIG['evaluation_strategy'],
        remove_unused_columns=TRAINING_CONFIG['remove_unused_columns'],
        dataloader_pin_memory=TRAINING_CONFIG['dataloader_pin_memory'],
        gradient_checkpointing=TRAINING_CONFIG['gradient_checkpointing'],
        max_grad_norm=TRAINING_CONFIG['max_grad_norm'],
        # Multi-GPU settings
        dataloader_num_workers=4,
        report_to=["tensorboard", "wandb"],  # Enable logging
        save_total_limit=2,  # Only save last 2 checkpoints
        load_best_model_at_end=True,
        metric_for_best_model="loss",
        greater_is_better=False
    )

    # 4. Initialize trainer
    trainer = SFTTrainer(
        model=model,
        args=training_args,
        train_dataset=train_dataset,
        eval_dataset=None,  # Using train dataset for now
        tokenizer=processor,
        dataset_text_field="text",  # Field in dataset containing text
        max_seq_length=512,  # Maximum sequence length
        formatting_func=None,  # Custom formatting handled in dataset
    )

    # 5. Start training
    print("🏃‍♂️ Starting training...")
    trainer.train()

    # 6. Save the fine-tuned model
    print("💾 Saving fine-tuned model...")
    model.save_pretrained(TRAINING_CONFIG['output_dir'])
    processor.save_pretrained(TRAINING_CONFIG['output_dir'])

    print(f"✅ Fine-tuning complete! Model saved to {TRAINING_CONFIG['output_dir']}")

    return model

if __name__ == "__main__":
    trained_model = run_vla_fine_tuning()
```

## 17.3 Implementing LoRA and QLoRA Techniques

Low-Rank Adaptation (LoRA) and Quantized LoRA (QLoRA) allow efficient fine-tuning of large models without modifying the full parameter space:

```python
import torch
import bitsandbytes as bnb
from peft import LoraConfig, get_peft_model, TaskType, prepare_model_for_kbit_training
from transformers import AutoModelForVision2Seq, AutoProcessor
from transformers import BitsAndBytesConfig

def setup_qlora_model(model_name: str):
    """
    Setup model with QLoRA (Quantized LoRA) for memory-efficient fine-tuning
    """
    # Configure quantization settings for QLoRA
    quantization_config = BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_use_double_quant=True,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_compute_dtype=torch.float16
    )

    # Load model with 4-bit quantization
    model = AutoModelForVision2Seq.from_pretrained(
        model_name,
        torch_dtype=torch.float16,
        quantization_config=quantization_config,
        device_map="auto",  # Distribute across available GPUs automatically
        trust_remote_code=True
    )

    # Prepare model for k-bit training
    model = prepare_model_for_kbit_training(model)

    # Configure LoRA
    lora_config = LoraConfig(
        task_type=TaskType.CAUSAL_LM,
        inference_mode=False,
        r=64,  # Rank of LoRA update matrices
        lora_alpha=16,  # Scaling factor
        lora_dropout=0.1,  # Dropout for LoRA layers
        target_modules=[
            "q_proj", "k_proj", "v_proj", "o_proj",  # Attention projection layers
            "gate_proj", "up_proj", "down_proj",     # Feed-forward layers
            "lm_head"  # Language model head
        ]
    )

    # Apply LoRA to the quantized model
    model = get_peft_model(model, lora_config)

    # Print trainable parameters
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    total_params = sum(p.numel() for p in model.parameters())

    print(f"Trainable params: {trainable_params:,}")
    print(f"Total params: {total_params:,}")
    print(f"Trainable %: {(trainable_params/total_params)*100:.2f}%")

    return model

def setup_multigpu_training(model, num_gpus: int = 4):
    """
    Setup model for multi-GPU training
    """
    # If we have multiple GPUs, use torch.nn.DataParallel or FSDP
    if num_gpus > 1:
        print(f"Setting up model for {num_gpus}-GPU training...")

        # Option 1: Using PyTorch DDP (recommended for VLA training)
        # This is handled by Accelerate in the training script

        # Option 2: Using FSDP for very large models
        # from torch.distributed.fsdp import FullyShardedDataParallel as FSDP
        # model = FSDP(model, device_id=torch.cuda.current_device())

    return model

def advanced_lora_configurations():
    """
    Different LoRA configurations for specific use cases
    """
    configs = {
        # 1. Default configuration for humanoid robotics
        "humanoid_default": {
            "r": 64,
            "lora_alpha": 16,
            "lora_dropout": 0.1,
            "target_modules": [
                "q_proj", "k_proj", "v_proj", "o_proj",
                "gate_proj", "up_proj", "down_proj", "lm_head"
            ],
            "use_dora": False  # Disable DoRA unless needed
        },

        # 2. Memory-efficient configuration for smaller GPUs
        "humanoid_efficient": {
            "r": 32,
            "lora_alpha": 16,
            "lora_dropout": 0.05,
            "target_modules": [
                "q_proj", "v_proj",  # Only query and value projections
                "up_proj", "down_proj"  # Feed-forward layers
            ],
            "use_dora": False
        },

        # 3. High-performance configuration for complex tasks
        "humanoid_high_performance": {
            "r": 128,
            "lora_alpha": 32,
            "lora_dropout": 0.1,
            "target_modules": [
                "q_proj", "k_proj", "v_proj", "o_proj",
                "gate_proj", "up_proj", "down_proj",
                "embed_tokens", "lm_head"  # Include embeddings and head
            ],
            "use_dora": True  # Use DoRA for better initialization
        }
    }

    return configs

def create_training_config(config_name: str = "humanoid_default"):
    """
    Create training configuration based on use case
    """
    configs = advanced_lora_configurations()

    if config_name not in configs:
        raise ValueError(f"Unknown config name: {config_name}")

    return configs[config_name]

def optimize_for_training(model):
    """
    Apply various optimizations for faster training
    """
    # Enable gradient checkpointing to save memory
    model.gradient_checkpointing_enable()

    # Optimize attention mechanisms if using newer architectures
    if hasattr(model, 'config') and hasattr(model.config, 'use_cache'):
        model.config.use_cache = False  # Disable KV cache for training

    # Apply torch.compile for additional speedup (PyTorch 2.0+)
    try:
        model = torch.compile(model, mode='reduce-overhead', fullgraph=True)
        print("Applied torch.compile optimization")
    except Exception as e:
        print(f"Torch compile not available or failed: {e}")

    return model
```

## 17.4 Generating High-Quality Training Data

Creating quality training data is crucial for successful VLA fine-tuning:

```python
import json
import numpy as np
from PIL import Image
import cv2
import os
from typing import List, Dict, Any, Tuple
import pickle

class VLADataGenerator:
    """
    Generate high-quality VLA training data with action chunking and language annotation
    """
    def __init__(self, isaac_recording_path: str, output_path: str):
        self.isaac_recording_path = isaac_recording_path
        self.output_path = output_path
        self.episode_metadata = []

    def generate_training_data(self) -> List[Dict[str, Any]]:
        """
        Main function to generate training data from Isaac Sim recordings
        """
        print("🚀 Starting VLA Training Data Generation from Isaac Sim recordings")

        # Get all episode directories
        episode_dirs = [
            d for d in os.listdir(self.isaac_recording_path)
            if os.path.isdir(os.path.join(self.isaac_recording_path, d))
        ]

        all_episodes_data = []
        failed_episodes = 0

        for i, episode_dir in enumerate(episode_dirs):
            print(f"Processing episode {i+1}/{len(episode_dirs)}: {episode_dir}")

            try:
                episode_data = self.process_episode(episode_dir)
                if episode_data:
                    all_episodes_data.extend(episode_data)
                    print(f"  ✓ Generated {len(episode_data)} samples")
                else:
                    failed_episodes += 1
                    print(f"  ✗ Failed to process episode")
            except Exception as e:
                failed_episodes += 1
                print(f"  ✗ Error processing episode {episode_dir}: {e}")
                continue

        print(f"Data generation complete: {len(all_episodes_data)} samples, {failed_episodes} failed episodes")
        return all_episodes_data

    def process_episode(self, episode_dir: str) -> List[Dict[str, Any]]:
        """
        Process a single Isaac Sim episode to extract training samples
        """
        episode_path = os.path.join(self.isaac_recording_path, episode_dir)

        # Load episode data
        data_path = os.path.join(episode_path, "episode_data.npz")
        metadata_path = os.path.join(episode_path, "metadata.json")

        if not os.path.exists(data_path) or not os.path.exists(metadata_path):
            print(f"Missing data files for episode {episode_dir}")
            return []

        # Load the data
        episode_data = np.load(data_path)
        with open(metadata_path, 'r') as f:
            metadata = json.load(f)

        # Extract components
        images = self.load_episode_images(episode_path)
        actions = episode_data['actions']  # Shape: [T, 23] for 23-DoF athena
        states = episode_data.get('states', np.zeros((len(actions), 47)))  # [T, state_dim]
        timestamps = episode_data.get('timestamps', np.arange(len(actions)))

        # Generate language annotations
        language_annotations = self.annotate_episode(metadata, actions, states)

        # Create training samples using action chunking
        training_samples = self.create_training_samples(
            images, actions, language_annotations, episode_dir
        )

        return training_samples

    def load_episode_images(self, episode_path: str) -> List[str]:
        """
        Load all image paths from an episode
        """
        image_dir = os.path.join(episode_path, "images")
        if not os.path.exists(image_dir):
            return []

        image_files = sorted([
            os.path.join(image_dir, f) for f in os.listdir(image_dir)
            if f.lower().endswith(('.png', '.jpg', '.jpeg'))
        ])

        return image_files

    def annotate_episode(self, metadata: Dict[str, Any], actions: np.ndarray,
                        states: np.ndarray) -> List[str]:
        """
        Generate natural language annotations for episode segments

        Args:
            metadata: Episode metadata from Isaac Sim
            actions: Action sequence [T, 23]
            states: State sequence [T, state_dim]

        Returns:
            List of language annotations for each time step
        """
        annotations = []
        task_description = metadata.get('task_description', 'perform a task')

        # Analyze action patterns to generate more specific annotations
        for t in range(len(actions)):
            # Analyze the action at time t to determine what the robot is doing
            action = actions[t]

            # Simple action classification based on joint movements
            annotation = self.classify_action_segment(action, states[t] if t < len(states) else None, task_description)
            annotations.append(annotation)

        return annotations

    def classify_action_segment(self, action: np.ndarray, state: np.ndarray,
                               base_task: str) -> str:
        """
        Classify what the robot is doing based on action vector

        Args:
            action: Action vector for current time step
            state: State vector for current time step
            base_task: Overall task the episode is demonstrating

        Returns:
            Natural language description of the action
        """
        # Analyze action to determine movement patterns
        abs_action = np.abs(action)

        # Detect major movement patterns
        leg_movement = abs_action[0:12].sum()  # Legs (first 12 DoF)
        arm_movement = abs_action[12:20].sum()  # Arms (DoF 13-20)
        head_torso_movement = abs_action[20:23].sum()  # Head/torso (DoF 21-23)

        # Generate specific annotations based on movement patterns
        if leg_movement > 0.3 and arm_movement < 0.2:
            if leg_movement > 0.6:  # Large leg movement
                return self.generate_walking_annotation(base_task)
            else:  # Smaller leg movement
                return self.generate_balancing_annotation(base_task)
        elif arm_movement > 0.3 and leg_movement < 0.2:
            if arm_movement > 0.5:  # Large arm movement
                return self.generate_manipulation_annotation(base_task)
            else:  # Smaller arm movement
                return self.generate_preparation_annotation(base_task)
        elif leg_movement > 0.2 and arm_movement > 0.2:
            # Combined movement
            return self.generate_combined_annotation(base_task)
        else:
            # Minimal movement (perhaps idle or fine adjustments)
            return self.generate_idle_annotation(base_task)

    def generate_walking_annotation(self, base_task: str) -> str:
        """Generate walking-related annotations"""
        variations = [
            f"walk forward to {base_task}",
            f"take steps toward completing {base_task}",
            f"move closer to the goal of {base_task}",
            f"advance toward the {base_task.replace('the ', '')} location"
        ]
        return np.random.choice(variations)

    def generate_balancing_annotation(self, base_task: str) -> str:
        """Generate balancing-related annotations"""
        variations = [
            f"maintain balance while preparing for {base_task}",
            f"adjust posture for {base_task}",
            f"stabilize stance for {base_task}",
            f"keep upright while working on {base_task}"
        ]
        return np.random.choice(variations)

    def generate_manipulation_annotation(self, base_task: str) -> str:
        """Generate manipulation-related annotations"""
        variations = [
            f"manipulate object for {base_task}",
            f"perform manipulation aspect of {base_task}",
            f"use hands to execute {base_task}",
            f"carry out manipulation part of {base_task}"
        ]
        return np.random.choice(variations)

    def generate_preparation_annotation(self, base_task: str) -> str:
        """Generate preparation-related annotations"""
        variations = [
            f"prepare to execute {base_task}",
            f"get ready to {base_task}",
            f"position for {base_task}",
            f"align for the {base_task} task"
        ]
        return np.random.choice(variations)

    def generate_combined_annotation(self, base_task: str) -> str:
        """Generate combined movement annotations"""
        variations = [
            f"coordinate movement to achieve {base_task}",
            f"perform coordinated action for {base_task}",
            f"execute multi-part movement for {base_task}",
            f"combine locomotion and manipulation for {base_task}"
        ]
        return np.random.choice(variations)

    def generate_idle_annotation(self, base_task: str) -> str:
        """Generate idle movement annotations"""
        variations = [
            f"observe and assess situation before {base_task}",
            f"wait and evaluate approach for {base_task}",
            f"monitor environment during {base_task}",
            f"pause between steps of {base_task}"
        ]
        return np.random.choice(variations)

    def create_training_samples(self, images: List[str], actions: np.ndarray,
                               annotations: List[str], episode_id: str) -> List[Dict[str, Any]]:
        """
        Create training samples with action chunking

        Args:
            images: List of image file paths
            actions: Action sequence [T, 23]
            annotations: Language annotations for each step
            episode_id: Episode identifier

        Returns:
            List of training samples ready for VLA training
        """
        training_samples = []

        # Use action chunking to create samples
        chunk_size = 1  # For per-timestep training
        stride = 1      # Overlap between chunks

        for start_idx in range(0, len(actions) - chunk_size + 1, stride):
            end_idx = start_idx + chunk_size

            # Create a sample for each time step in the chunk
            for t in range(start_idx, end_idx):
                if t < len(images) and t < len(annotations):
                    sample = {
                        'image_path': images[t],
                        'instruction': annotations[t],
                        'action': actions[t].tolist(),  # Convert to list for JSON serialization
                        'episode_id': episode_id,
                        'timestep': t,
                        'full_action_sequence': actions[start_idx:end_idx].flatten().tolist()
                    }

                    # Format for VLA training
                    sample['formatted_prompt'] = (
                        f"In: What action should the robot take to {sample['instruction'].lower()}?\n"
                        f"Out: {np.array2string(np.array(sample['action']), separator=',')}"
                    )

                    training_samples.append(sample)

        return training_samples

def save_training_dataset(dataset: List[Dict[str, Any]], output_path: str):
    """
    Save the training dataset in a format suitable for HuggingFace datasets
    """
    import pandas as pd
    from datasets import Dataset

    # Convert to DataFrame
    df = pd.DataFrame(dataset)

    # Create HuggingFace Dataset
    hf_dataset = Dataset.from_pandas(df)

    # Save dataset
    hf_dataset.save_to_disk(output_path)
    print(f"Training dataset saved to {output_path}")

def run_data_generation_pipeline():
    """
    Execute the complete data generation pipeline
    """
    print("🚀 Starting VLA Data Generation Pipeline")

    # Initialize data generator
    generator = VLADataGenerator(
        isaac_recording_path="isaac_sim_recordings/athena_episodes",
        output_path="vla_training_data/athena_vla_dataset"
    )

    # Generate training data
    training_data = generator.generate_training_data()

    if training_data:
        # Save the dataset
        save_training_dataset(training_data, generator.output_path)

        print(f"✅ Data generation completed with {len(training_data)} training samples")

        # Print statistics
        unique_episodes = set(sample['episode_id'] for sample in training_data)
        print(f"  Episodes: {len(unique_episodes)}")
        print(f"  Total samples: {len(training_data)}")
        print(f"  Average samples per episode: {len(training_data)/len(unique_episodes):.1f}")

        return training_data
    else:
        print("❌ No training data was generated")
        return []

if __name__ == "__main__":
    training_data = run_data_generation_pipeline()
```

## 17.5 Optimizing for Jetson Orin Deployment

To deploy our fine-tuned model on Jetson Orin with the required performance:

```python
import torch
import onnx
import numpy as np

def export_model_for_jetson(model_path: str, output_path: str,
                           quantization_mode: str = "int8"):
    """
    Export fine-tuned VLA model for efficient Jetson deployment

    Args:
        model_path: Path to fine-tuned model
        output_path: Output path for optimized model
        quantization_mode: Quantization mode ("int8", "fp16", or "fp32")
    """
    print(f"Exporting model for Jetson deployment with {quantization_mode} quantization")

    # Load the trained PEFT model
    from peft import PeftModel
    from transformers import AutoModelForVision2Seq, AutoProcessor

    # Load base model
    base_model = AutoModelForVision2Seq.from_pretrained(
        "openvla/openvla-7b",  # Original base model
        torch_dtype=torch.float16 if quantization_mode != "int8" else torch.float32,
        low_cpu_mem_usage=True,
        device_map="auto"
    )

    # Load PEFT adapter
    model = PeftModel.from_pretrained(base_model, model_path)

    # Merge the adapter with the base model
    model = model.merge_and_unload()

    # Set to evaluation mode
    model.eval()

    # Create dummy inputs for exporting
    dummy_image = torch.randn(1, 3, 224, 224, dtype=torch.float32)  # Example image input
    dummy_input_ids = torch.randint(0, 32000, (1, 512))  # Example text input
    dummy_attention_mask = torch.ones((1, 512))
    dummy_pixel_values = torch.randn(1, 3, 224, 224, dtype=torch.float32)

    # Export to ONNX
    onnx_path = output_path.replace(".engine", ".onnx")

    torch.onnx.export(
        model,
        (dummy_input_ids, dummy_attention_mask, dummy_pixel_values),
        onnx_path,
        export_params=True,
        opset_version=17,  # Latest opset for better optimization
        do_constant_folding=True,
        input_names=['input_ids', 'attention_mask', 'pixel_values'],
        output_names=['logits'],
        dynamic_axes={
            'input_ids': {0: 'batch_size', 1: 'sequence'},
            'attention_mask': {0: 'batch_size', 1: 'sequence'},
            'pixel_values': {0: 'batch_size', 1: 'channels', 2: 'height', 3: 'width'},
            'logits': {0: 'batch_size', 1: 'sequence'}
        }
    )

    print(f"ONNX model exported to {onnx_path}")

    # Further optimize for Jetson using TensorRT if available
    try:
        import tensorrt as trt

        # Create TensorRT builder
        logger = trt.Logger(trt.Logger.WARNING)
        builder = trt.Builder(logger)
        network = builder.create_network(
            1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        )
        config = builder.create_builder_config()

        # Parse ONNX to TensorRT network
        parser = trt.OnnxParser(network, logger)
        success = parser.parse_from_file(onnx_path)

        if not success:
            print("❌ Failed to parse ONNX model for TensorRT")
            for idx in range(parser.num_errors):
                print(parser.get_error(idx))
            return onnx_path  # Return ONNX model as fallback

        # Set optimization profile
        profile = builder.create_optimization_profile()

        # Define input shapes - adjust based on your model's requirements
        profile.set_shape("input_ids", (1, 1), (1, 256), (1, 512))
        profile.set_shape("attention_mask", (1, 1), (1, 256), (1, 512))
        profile.set_shape("pixel_values", (1, 3, 224, 224), (1, 3, 224, 224), (1, 3, 224, 224))

        config.add_optimization_profile(profile)

        # Apply quantization
        if quantization_mode == "int8":
            config.set_flag(trt.BuilderFlag.INT8)
            # Calibration would be needed here for INT8
        elif quantization_mode == "fp16":
            config.set_flag(trt.BuilderFlag.FP16)

        # Build engine
        serialized_engine = builder.build_serialized_network(network, config)

        # Save engine
        with open(output_path, "wb") as f:
            f.write(serialized_engine)

        print(f"✅ TensorRT engine saved to {output_path}")
        return output_path

    except ImportError:
        print("⚠️  TensorRT not available, using ONNX model as fallback")
        return onnx_path  # Return ONNX model as fallback
    except Exception as e:
        print(f"TensorRT optimization failed: {e}")
        return onnx_path  # Return ONNX model as fallback

def benchmark_model_performance(model_path: str, device="cuda"):
    """
    Benchmark model performance to ensure it meets Jetson requirements

    Args:
        model_path: Path to the model to benchmark
        device: Device to benchmark on

    Returns:
        Performance metrics
    """
    import onnxruntime as ort

    # Create ONNX Runtime session for performance testing
    session = ort.InferenceSession(
        model_path,
        providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
    )

    # Prepare dummy inputs
    dummy_input_ids = np.random.randint(0, 32000, size=(1, 512)).astype(np.int32)
    dummy_attention_mask = np.ones((1, 512), dtype=np.int32)
    dummy_pixel_values = np.random.randn(1, 3, 224, 224).astype(np.float32)

    # Warm up
    for _ in range(10):
        _ = session.run(None, {
            'input_ids': dummy_input_ids,
            'attention_mask': dummy_attention_mask,
            'pixel_values': dummy_pixel_values
        })

    # Benchmark
    import time
    times = []

    for _ in range(100):
        start_time = time.time()
        _ = session.run(None, {
            'input_ids': dummy_input_ids,
            'attention_mask': dummy_attention_mask,
            'pixel_values': dummy_pixel_values
        })
        end_time = time.time()
        times.append(end_time - start_time)

    # Calculate metrics
    avg_time = np.mean(times)
    p95_time = np.percentile(times, 95)
    p99_time = np.percentile(times, 99)

    tokens_per_sec = 1.0 / avg_time

    metrics = {
        'avg_inference_time_s': avg_time,
        'avg_inference_time_ms': avg_time * 1000,
        'p95_inference_time_ms': p95_time * 1000,
        'p99_inference_time_ms': p99_time * 1000,
        'tokens_per_second': tokens_per_sec,
        'inference_throughput': tokens_per_sec * 23,  # 23 DoF actions per token generation
        'meets_jetson_requirement': tokens_per_sec >= 8.0  # 8 tokens/sec minimum for Jetson
    }

    print(f"\n📊 PERFORMANCE BENCHMARK RESULTS")
    print(f"Average inference time: {metrics['avg_inference_time_ms']:.2f} ms")
    print(f"P95 inference time: {metrics['p95_inference_time_ms']:.2f} ms")
    print(f"Tokens per second: {metrics['tokens_per_second']:.2f}")
    print(f"Throughput: {metrics['inference_throughput']:.2f} DoF-actions/sec")
    print(f"Meets Jetson requirement (≥8 tok/s): {'✅ YES' if metrics['meets_jetson_requirement'] else '❌ NO'}")

    return metrics

# Example usage for deployment optimization
def optimize_for_jetson_deployment():
    """
    Complete optimization pipeline for Jetson Orin deployment
    """
    print("🔧 Optimizing VLA model for Jetson Orin 16GB deployment")

    # Export model with INT8 quantization for maximum efficiency
    optimized_model_path = export_model_for_jetson(
        model_path="./fine_tuned_athena_vla/checkpoint-final",
        output_path="./deployment_models/athena_vla_jetson.engine",
        quantization_mode="int8"
    )

    # Benchmark performance
    performance_metrics = benchmark_model_performance(optimized_model_path)

    # Check if targets are met
    if performance_metrics['meets_jetson_requirement']:
        print("✅ Model meets Jetson Orin performance targets!")

        # Generate deployment configuration
        deployment_config = {
            'model_path': optimized_model_path,
            'input_shapes': {
                'image': [1, 3, 224, 224],
                'text': [1, 512]
            },
            'performance_target': {
                'tokens_per_sec': 8,
                'max_latency_ms': 125,  # 1/8 tokens per sec = 125ms per token
                'achieved_tokens_per_sec': performance_metrics['tokens_per_second']
            },
            'hardware_target': 'jetson_orin_16gb',
            'optimization_technique': 'int8_tensorrt'
        }

        # Save deployment config
        import json
        with open('./deployment_models/deployment_config.json', 'w') as f:
            json.dump(deployment_config, f, indent=2)

        print("📄 Deployment configuration saved")
        return True
    else:
        print("❌ Model does not meet Jetson performance targets, further optimization needed")
        print(f"Current performance: {performance_metrics['tokens_per_second']:.2f} tok/s, Target: 8+ tok/s")
        return False

if __name__ == "__main__":
    success = optimize_for_jetson_deployment()
    if success:
        print("\n🎉 VLA model successfully optimized for Jetson Orin deployment!")
    else:
        print("\n⚠️  Further optimization may be required to meet performance targets")
```

## 17.6 Chapter Summary

This chapter has provided you with a complete pipeline for fine-tuning Vision-Language-Action models specifically for your "athena" humanoid robot. We covered:

1. Setting up the fine-tuning environment with appropriate dependencies
2. Implementing LoRA and QLoRA techniques for efficient training on limited hardware
3. Creating high-quality training data from Isaac Sim recordings with action chunking
4. Optimizing models for deployment on resource-constrained platforms like Jetson Orin
5. Validating that the optimized model meets the performance requirements (8-12 tokens/sec)

The techniques covered in this chapter enable you to create specialized VLA models that understand your specific robot "athena" and environment, significantly outperforming generic models on your particular tasks.

## End-of-Chapter Exercises

1. Fine-tune OpenVLA-7B on your Isaac Sim dataset using the provided pipeline
2. Experiment with different LoRA configurations to find the optimal balance of performance and training time
3. Generate a high-quality VLA dataset from your Isaac Sim recordings using action chunking
4. Optimize your fine-tuned model for deployment on Jetson Orin and validate performance targets
5. Compare the performance of your fine-tuned model against the generic baseline on your robot tasks
6. Implement additional safety constraints in your model's action generation