# Chapter 9: Domain Randomization and Large-Scale Synthetic Data Generation

## Learning Objectives
By the end of this chapter, you will be able to:
- Implement domain randomization scripts for lighting, textures, and physics parameters
- Generate COCO, YOLO, and OpenVLA-compatible datasets at scale
- Randomize robot appearance, physics parameters, and scene environments
- Create a Python API for scripted data collection

## 9.1 Domain Randomization Theory and Applications

Domain randomization is a powerful technique to improve the transfer of models trained in simulation to the real world. The core principle is to randomize various aspects of the simulation environment so that the trained model becomes robust to variations it might encounter in reality.

For our Athena humanoid, domain randomization can be applied to:
- Lighting conditions
- Textures and materials
- Physics parameters (friction, damping, etc.)
- Scene layouts and backgrounds
- Robot appearance and properties
- Sensor noise characteristics

## 9.2 Lighting and Material Randomization Script

```python
#!/usr/bin/env python3
"""
Domain randomization for lighting and materials in simulation
"""
import random
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3, Point
from sensor_msgs.msg import CameraInfo


class LightingRandomizer(Node):
    def __init__(self):
        super().__init__('lighting_randomizer')
        
        # Publishers for lighting and material changes
        self.lighting_cmd_publisher = self.create_publisher(Bool, '/randomize_lighting', 10)
        self.material_cmd_publisher = self.create_publisher(Bool, '/randomize_materials', 10)
        
        # Timer to periodically randomize
        self.randomization_timer = self.create_timer(10.0, self.randomize_environment)
        
        self.get_logger().info('Lighting and Material Randomizer initialized')

    def randomize_environment(self):
        """Randomize lighting and materials in the scene"""
        # Randomize main light direction and intensity
        main_light_dir = Vector3()
        main_light_dir.x = random.uniform(-1.0, 1.0)
        main_light_dir.y = random.uniform(-1.0, 1.0)
        main_light_dir.z = random.uniform(-1.0, 0.0)  # Always pointing down-ish
        
        main_light_intensity = random.uniform(0.5, 1.5)
        
        # Randomize ambient light color
        ambient_color = [
            random.uniform(0.1, 0.3),
            random.uniform(0.1, 0.3),
            random.uniform(0.1, 0.3)
        ]
        
        self.get_logger().info(f'Randomized lighting: dir=({main_light_dir.x:.2f}, {main_light_dir.y:.2f}, {main_light_dir.z:.2f}), '
                              f'intensity={main_light_intensity:.2f}, ambient={ambient_color}')
        
        # In a real implementation, send commands to Gazebo/Unity to update lighting
        # For now, we'll just publish a command to trigger the randomization
        cmd_msg = Bool()
        cmd_msg.data = True
        self.lighting_cmd_publisher.publish(cmd_msg)
        self.material_cmd_publisher.publish(cmd_msg)


class TextureRandomizer(Node):
    def __init__(self):
        super().__init__('texture_randomizer')
        
        # Predefined texture library
        self.texture_library = [
            'textures/floor/marble_1.jpg', 'textures/floor/wood_1.jpg', 'textures/floor/concrete_1.jpg',
            'textures/floor/tile_1.jpg', 'textures/floor/metal_1.jpg', 'textures/floor/grass_1.jpg',
            'textures/wall/brick_1.jpg', 'textures/wall/paint_1.jpg', 'textures/wall/stone_1.jpg',
            'textures/objects/plastic_1.jpg', 'textures/objects/metal_1.jpg', 'textures/objects/wood_1.jpg'
        ]
        
        # Timer to periodically randomize textures
        self.randomization_timer = self.create_timer(15.0, self.randomize_textures)
        
        self.get_logger().info('Texture Randomizer initialized')

    def randomize_textures(self):
        """Randomize textures in the scene"""
        # Select random floor texture
        floor_texture = random.choice(self.texture_library[:6])  # Floor textures
        
        # Select random wall texture
        wall_texture = random.choice(self.texture_library[6:9])  # Wall textures
        
        # Select random object textures
        object_textures = [random.choice(self.texture_library[9:]) for _ in range(3)]
        
        self.get_logger().info(f'Randomized textures: floor={floor_texture}, walls={wall_texture}, objects={object_textures}')
        
        # In a real implementation, this would update materials in the simulation environment


class PhysicsRandomizer(Node):
    def __init__(self):
        super().__init__('physics_randomizer')
        
        # Physics parameters to randomize
        self.physics_params = {
            'ground_friction': [0.5, 1.5],      # Coefficient of friction range
            'ground_bounce': [0.0, 0.1],        # Restitution coefficient range
            'joint_damping': [0.01, 0.1],       # Joint damping range
            'joint_friction': [0.0, 0.05],      # Joint friction range
        }
        
        # Timer to periodically randomize physics
        self.randomization_timer = self.create_timer(20.0, self.randomize_physics)
        
        self.get_logger().info('Physics Randomizer initialized')

    def randomize_physics(self):
        """Randomize physics parameters in the simulation"""
        # Generate random values for each parameter
        randomized_params = {}
        for param, (min_val, max_val) in self.physics_params.items():
            randomized_params[param] = random.uniform(min_val, max_val)
        
        self.get_logger().info(f'Randomized physics: {randomized_params}')
        
        # In a real implementation, send these parameters to the physics engine
        # This could involve updating SDF files or sending commands to Gazebo
        # For now, we'll just log the changes
```

## 9.3 COCO Dataset Generator

```python
#!/usr/bin/env python3
"""
COCO dataset generator for synthetic data
"""
import json
import cv2
import numpy as np
from pycocotools.coco import COCO
import os
from datetime import datetime
import uuid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class COCODatasetGenerator:
    def __init__(self, output_dir, dataset_name="synthetic_athena"):
        self.output_dir = output_dir
        self.dataset_name = dataset_name
        self.bridge = CvBridge()
        
        # Initialize COCO dataset structure
        self.dataset = {
            "info": {
                "description": f"{dataset_name} Dataset",
                "url": "http://physical-ai-book.com",
                "version": "1.0",
                "year": 2025,
                "contributor": "Physical AI Research Team",
                "date_created": datetime.now().isoformat()
            },
            "licenses": [{
                "id": 1,
                "name": "Attribution-NonCommercial-ShareAlike License",
                "url": "http://creativecommons.org/licenses/by-nc-sa/2.0/"
            }],
            "categories": [],
            "images": [],
            "annotations": []
        }
        
        # Define categories (objects we want to detect)
        self.categories = [
            {"id": 1, "name": "athena_humanoid", "supercategory": "robot"},
            {"id": 2, "name": "cup", "supercategory": "object"},
            {"id": 3, "name": "chair", "supercategory": "furniture"},
            {"id": 4, "name": "table", "supercategory": "furniture"},
            {"id": 5, "name": "ball", "supercategory": "object"}
        ]
        
        self.dataset["categories"] = self.categories
        
        # Initialize counters
        self.image_id_counter = 1
        self.annotation_id_counter = 1
        
        # Create output directories
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "annotations"), exist_ok=True)

    def add_image(self, image_msg, camera_info_msg=None):
        """Add an image to the dataset"""
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        
        # Generate unique image filename
        image_filename = f"{self.image_id_counter:06d}.jpg"
        image_path = os.path.join(self.output_dir, "images", image_filename)
        
        # Save image
        cv2.imwrite(image_path, cv_image)
        
        # Add image info to dataset
        image_info = {
            "id": self.image_id_counter,
            "width": image_msg.width,
            "height": image_msg.height,
            "file_name": image_filename,
            "license": 1,
            "flickr_url": "",
            "coco_url": "",
            "date_captured": datetime.now().isoformat()
        }
        
        self.dataset["images"].append(image_info)
        
        return self.image_id_counter

    def add_annotation(self, image_id, category_id, bbox, segmentation=None, area=None):
        """Add an annotation (object detection) to the dataset"""
        if area is None:
            area = bbox[2] * bbox[3]  # width * height
        
        annotation = {
            "id": self.annotation_id_counter,
            "image_id": image_id,
            "category_id": category_id,
            "bbox": bbox,  # [x, y, width, height]
            "area": area,
            "iscrowd": 0
        }
        
        if segmentation:
            annotation["segmentation"] = segmentation
        
        self.dataset["annotations"].append(annotation)
        self.annotation_id_counter += 1

    def save_dataset(self):
        """Save the COCO dataset to JSON file"""
        filename = f"{self.dataset_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        filepath = os.path.join(self.output_dir, "annotations", filename)
        
        with open(filepath, 'w') as f:
            json.dump(self.dataset, f, indent=2)
        
        print(f"Dataset saved to {filepath}")
        print(f"Total images: {len(self.dataset['images'])}")
        print(f"Total annotations: {len(self.dataset['annotations'])}")

    def add_athena_annotations(self, image_id, athena_position, athena_size):
        """Add annotations for Athena humanoid in the image"""
        # Assuming athena_position is (x, y) and athena_size is (width, height)
        bbox = [athena_position[0], athena_position[1], athena_size[0], athena_size[1]]
        
        # Calculate area
        area = athena_size[0] * athena_size[1]
        
        # Add annotation for Athena
        self.add_annotation(
            image_id=image_id,
            category_id=1,  # athena_humanoid
            bbox=bbox,
            area=area
        )


class COCODataCollector(Node):
    def __init__(self):
        super().__init__('coco_data_collector')
        
        # Parameters
        self.declare_parameter('output_dir', '/data/coco_dataset')
        self.declare_parameter('max_images', 100000)  # 100k images goal
        
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.max_images = self.get_parameter('max_images').get_parameter_value().integer_value
        
        # Initialize COCO generator
        self.coco_generator = COCODatasetGenerator(self.output_dir, "athena_2025")
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/athena/camera/image_raw', self.image_callback, 10)
        
        # Counter for collected images
        self.image_count = 0
        
        # Timer to save dataset periodically
        self.save_timer = self.create_timer(300.0, self.save_periodically)  # Every 5 minutes
        
        self.get_logger().info(f'COCO Data Collector initialized. Will collect up to {self.max_images} images.')

    def image_callback(self, msg):
        """Process incoming image and add to COCO dataset"""
        if self.image_count >= self.max_images:
            self.get_logger().info('Max image count reached, stopping collection.')
            return
        
        try:
            # Add image to dataset (in a real implementation, we'd also add annotations)
            image_id = self.coco_generator.add_image(msg)
            
            # In a real implementation, we would use computer vision or ground truth
            # to identify objects and add appropriate annotations
            # For now, we'll just add a placeholder annotation for Athena if detected
            
            # Placeholder: assume we can detect Athena in the center of the image
            # with some fixed bounding box
            if self.image_count % 10 == 0:  # Add annotation every 10th image
                # Calculate approximate position of Athena (center of image)
                center_x = msg.width // 2
                center_y = msg.height // 2
                width = msg.width // 4
                height = msg.height // 2
                
                self.coco_generator.add_athena_annotations(
                    image_id, 
                    (center_x - width//2, center_y - height//2), 
                    (width, height)
                )
            
            self.image_count += 1
            
            if self.image_count % 1000 == 0:
                self.get_logger().info(f'Collected {self.image_count}/{self.max_images} images')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def save_periodically(self):
        """Save the dataset periodically"""
        if self.image_count > 0:
            self.coco_generator.save_dataset()
    
    def destroy_node(self):
        """Save final dataset when node is destroyed"""
        if self.image_count > 0:
            self.coco_generator.save_dataset()
        super().destroy_node()
```

## 9.4 YOLO Dataset Generator

```python
#!/usr/bin/env python3
"""
YOLO dataset generator for synthetic data
"""
import os
import yaml
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class YOLODatasetGenerator:
    def __init__(self, output_dir, dataset_name="synthetic_athena"):
        self.output_dir = output_dir
        self.dataset_name = dataset_name
        self.bridge = CvBridge()
        
        # Create required directory structure
        self.images_dir = os.path.join(output_dir, "images")
        self.labels_dir = os.path.join(output_dir, "labels")
        self.train_dir = os.path.join(self.images_dir, "train")
        self.val_dir = os.path.join(self.images_dir, "val")
        self.test_dir = os.path.join(self.images_dir, "test")
        
        for dir_path in [self.images_dir, self.labels_dir, self.train_dir, 
                         self.val_dir, self.test_dir]:
            os.makedirs(dir_path, exist_ok=True)
        
        # YOLO classes (same as COCO for consistency)
        self.classes = {
            0: "athena_humanoid",
            1: "cup",
            2: "chair",
            3: "table",
            4: "ball"
        }
        
        # Create data.yaml file
        self.create_data_yaml()
        
        # Counter for image ID
        self.image_id = 0
    
    def create_data_yaml(self):
        """Create YOLO data.yaml file"""
        data = {
            'path': self.output_dir,
            'train': 'images/train',
            'val': 'images/val',
            'test': 'images/test',
            'nc': len(self.classes),
            'names': list(self.classes.values())
        }
        
        yaml_path = os.path.join(self.output_dir, 'data.yaml')
        with open(yaml_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
    
    def add_image(self, image_msg, objects_data, split='train'):
        """Add an image to YOLO dataset format"""
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        
        # Determine image save path based on split
        if split == 'train':
            img_save_path = os.path.join(self.train_dir, f"{self.image_id:06d}.jpg")
            label_save_path = os.path.join(self.labels_dir, f"{self.image_id:06d}.txt")
        elif split == 'val':
            img_save_path = os.path.join(self.val_dir, f"{self.image_id:06d}.jpg")
            label_save_path = os.path.join(self.labels_dir, f"{self.image_id:06d}.txt")
        else:
            img_save_path = os.path.join(self.test_dir, f"{self.image_id:06d}.jpg")
            label_save_path = os.path.join(self.labels_dir, f"{self.image_id:06d}.txt")
        
        # Save image
        cv2.imwrite(img_save_path, cv_image)
        
        # Create YOLO format label file
        # YOLO format: class_id center_x center_y width height (normalized 0-1)
        label_lines = []
        for obj in objects_data:
            class_id = obj['class_id']
            bbox = obj['bbox']  # [x, y, width, height] in pixels
            
            # Convert to YOLO format (normalized center coordinates)
            center_x = (bbox[0] + bbox[2] / 2) / image_msg.width
            center_y = (bbox[1] + bbox[3] / 2) / image_msg.height
            width = bbox[2] / image_msg.width
            height = bbox[3] / image_msg.height
            
            label_lines.append(f"{class_id} {center_x:.6f} {center_y:.6f} {width:.6f} {height:.6f}")
        
        # Write label file
        with open(label_save_path, 'w') as f:
            f.write('\n'.join(label_lines))
        
        self.image_id += 1
        
        return img_save_path, label_save_path


class YOLODataCollector(Node):
    def __init__(self):
        super().__init__('yolo_data_collector')
        
        # Parameters
        self.declare_parameter('output_dir', '/data/yolo_dataset')
        self.declare_parameter('max_images', 100000)
        
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.max_images = self.get_parameter('max_images').get_parameter_value().integer_value
        
        # Initialize YOLO generator
        self.yolo_generator = YOLODatasetGenerator(self.output_dir, "athena_2025")
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/athena/camera/image_raw', self.image_callback, 10)
        
        # Counter for collected images
        self.image_count = 0
        
        self.get_logger().info(f'YOLO Data Collector initialized. Will collect up to {self.max_images} images.')

    def image_callback(self, msg):
        """Process incoming image and add to YOLO dataset"""
        if self.image_count >= self.max_images:
            self.get_logger().info('Max image count reached, stopping collection.')
            return
        
        try:
            # In a real implementation, we would use ground truth or detection
            # to identify objects and their bounding boxes
            # For now, we'll add a placeholder with Athena detection
            objects_data = []
            if self.image_count % 5 == 0:  # Add Athena every 5th image
                # Assume Athena is in the center
                center_x = msg.width // 2
                center_y = msg.height // 2
                width = msg.width // 4
                height = msg.height // 2
                
                objects_data.append({
                    'class_id': 0,  # athena_humanoid
                    'bbox': [center_x - width//2, center_y - height//2, width, height]
                })
            
            # Add image to YOLO dataset
            img_path, label_path = self.yolo_generator.add_image(msg, objects_data)
            
            self.image_count += 1
            
            if self.image_count % 1000 == 0:
                self.get_logger().info(f'Collected {self.image_count}/{self.max_images} images for YOLO')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image for YOLO: {e}')

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        super().destroy_node()
```

## 9.5 OpenVLA Dataset Generator

```python
#!/usr/bin/env python3
"""
OpenVLA dataset generator for multimodal visuomotor manipulation
"""
import os
import json
import numpy as np
import cv2
from sensor_msgs.msg import Image, JointState, PointCloud2
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import tf2_ros
from rclpy.time import Time


class OpenVLADatasetGenerator:
    def __init__(self, output_dir, dataset_name="synthetic_athena_openvla"):
        self.output_dir = output_dir
        self.dataset_name = dataset_name
        self.bridge = CvBridge()
        
        # Create directory structure for OpenVLA
        os.makedirs(os.path.join(output_dir, "episodes"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "meta"), exist_ok=True)
        
        # Episode counter
        self.episode_id = 0
        self.step_id = 0
        
        # Store episode data
        self.current_episode = []

    def start_episode(self):
        """Start a new episode"""
        self.episode_id += 1
        self.step_id = 0
        self.current_episode = []
        print(f"Starting episode {self.episode_id}")

    def add_step(self, image_msg, joint_state, action, end_effector_pose=None):
        """Add a step to the current episode"""
        step_data = {
            "step_id": self.step_id,
            "timestamp": self.step_id * 0.1,  # Assume 10Hz control frequency
            "image": self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8').tolist(),
            "joint_positions": joint_state.position,
            "joint_velocities": joint_state.velocity if joint_state.velocity else [0.0] * len(joint_state.position),
            "joint_efforts": joint_state.effort if joint_state.effort else [0.0] * len(joint_state.position),
            "action": action.tolist() if isinstance(action, np.ndarray) else action,
        }
        
        if end_effector_pose:
            step_data["end_effector_pose"] = {
                "position": [end_effector_pose.position.x, end_effector_pose.position.y, end_effector_pose.position.z],
                "orientation": [end_effector_pose.orientation.x, end_effector_pose.orientation.y, 
                               end_effector_pose.orientation.z, end_effector_pose.orientation.w]
            }
        
        self.current_episode.append(step_data)
        self.step_id += 1

    def end_episode(self):
        """End current episode and save to disk"""
        if not self.current_episode:
            return
        
        # Create episode file
        episode_filename = f"episode_{self.episode_id:06d}.json"
        episode_path = os.path.join(self.output_dir, "episodes", episode_filename)
        
        episode_data = {
            "episode_id": self.episode_id,
            "steps": self.current_episode,
            "total_steps": len(self.current_episode),
        }
        
        with open(episode_path, 'w') as f:
            json.dump(episode_data, f, indent=2)
        
        print(f"Episode {self.episode_id} saved with {len(self.current_episode)} steps")
        
        # Clear current episode
        self.current_episode = []


class OpenVLADataCollector(Node):
    def __init__(self):
        super().__init__('openvla_data_collector')
        
        # Parameters
        self.declare_parameter('output_dir', '/data/openvla_dataset')
        self.declare_parameter('max_episodes', 1000)
        self.declare_parameter('steps_per_episode', 50)
        
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.max_episodes = self.get_parameter('max_episodes').get_parameter_value().integer_value
        self.steps_per_episode = self.get_parameter('steps_per_episode').get_parameter_value().integer_value
        
        # Initialize OpenVLA generator
        self.openvla_generator = OpenVLADatasetGenerator(self.output_dir, "athena_2025_openvla")
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/athena/camera/image_raw', self.image_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/athena/joint_states', self.joint_state_callback, 10)
        
        # Store the latest joint state
        self.latest_joint_state = None
        
        # Episode management
        self.episode_count = 0
        self.steps_in_current_episode = 0
        
        # Timer to start new episodes
        self.episode_timer = self.create_timer(0.1, self.episode_callback)
        
        self.get_logger().info(f'OpenVLA Data Collector initialized. Will collect up to {self.max_episodes} episodes.')

    def joint_state_callback(self, msg):
        """Store the latest joint state"""
        self.latest_joint_state = msg

    def image_callback(self, msg):
        """Process incoming image"""
        # Store image for next episode step (if we have a joint state)
        if self.latest_joint_state:
            # In a real implementation, we would have the action that was executed
            # to reach this state. For now, we'll use a placeholder action
            action = np.random.random(23)  # 23-DoF for Athena
            
            # For end effector pose, we would get this from TF or inverse kinematics
            end_effector_pose = None  # Placeholder
            
            if self.steps_in_current_episode < self.steps_per_episode:
                self.openvla_generator.add_step(
                    msg, 
                    self.latest_joint_state, 
                    action, 
                    end_effector_pose
                )
                self.steps_in_current_episode += 1

    def episode_callback(self):
        """Handle episode management"""
        if self.steps_in_current_episode >= self.steps_per_episode:
            # End current episode
            self.openvla_generator.end_episode()
            self.steps_in_current_episode = 0
            self.episode_count += 1
            
            if self.episode_count < self.max_episodes:
                # Start new episode
                self.openvla_generator.start_episode()
            else:
                self.get_logger().info(f'Collected {self.episode_count} episodes. Stopping collection.')
                # Cancel the timer
                self.episode_timer.cancel()

    def destroy_node(self):
        """End current episode if node is destroyed"""
        if self.steps_in_current_episode > 0:
            self.openvla_generator.end_episode()
        super().destroy_node()
```

## 9.6 Complete Data Generation Pipeline

```python
#!/usr/bin/env python3
"""
Complete domain randomization and synthetic data generation pipeline
"""
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_noise_simulator import SensorNoiseSimulator
from coco_data_collector import COCODataCollector
from yolo_data_collector import YOLODataCollector
from openvla_data_collector import OpenVLADataCollector
from lighting_randomizer import LightingRandomizer, TextureRandomizer, PhysicsRandomizer


def main(args=None):
    rclpy.init(args=args)
    
    # Create nodes for the complete pipeline
    sensor_noise_simulator = SensorNoiseSimulator()
    coco_collector = COCODataCollector()
    yolo_collector = YOLODataCollector()
    openvla_collector = OpenVLADataCollector()
    
    # Environment randomizers
    lighting_randomizer = LightingRandomizer()
    texture_randomizer = TextureRandomizer()
    physics_randomizer = PhysicsRandomizer()
    
    # Create multi-threaded executor to run all nodes simultaneously
    executor = MultiThreadedExecutor(num_threads=8)
    
    # Add all nodes to executor
    executor.add_node(sensor_noise_simulator)
    executor.add_node(coco_collector)
    executor.add_node(yolo_collector)
    executor.add_node(openvla_collector)
    executor.add_node(lighting_randomizer)
    executor.add_node(texture_randomizer)
    executor.add_node(physics_randomizer)
    
    try:
        print('Starting domain randomization and synthetic data generation pipeline...')
        print('Collecting data in COCO, YOLO, and OpenVLA formats simultaneously...')
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        sensor_noise_simulator.destroy_node()
        coco_collector.destroy_node()
        yolo_collector.destroy_node()
        openvla_collector.destroy_node()
        lighting_randomizer.destroy_node()
        texture_randomizer.destroy_node()
        physics_randomizer.destroy_node()
        
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## "Pro Tips" Sidebar

- **Balanced Randomization**: Don't randomize too broadly or too narrowly - find the sweet spot that improves real-world transfer without degrading simulation performance.
- **Monitoring**: Track dataset statistics to ensure your randomization is producing the desired distribution of examples.
- **Validation**: Always test your randomized model on a small set of real data to verify the sim-to-real transfer.

## References to Official Documentation

- [PyTorch Domain Randomization](https://pytorch.org/tutorials/intermediate/ddp_tutorial.html)
- [COCO Dataset Format](https://cocodataset.org/#format-data)
- [YOLO Dataset Format](https://docs.ultralytics.com/datasets/detect/)

In the next chapter, we'll close the sim loop by integrating navigation, manipulation, and speech recognition in a complete autonomous digital twin.