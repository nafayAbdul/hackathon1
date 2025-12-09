"""
VLA Inference with Single Image Input
Listing 16.2: VLA inference with single image input
"""
import torch
import numpy as np
from PIL import Image
from transformers import AutoModel, AutoProcessor
from typing import Union, List, Optional, Tuple, Dict, Any
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)


class VLAInference:
    """
    Class to handle VLA inference with a single image input
    """
    
    def __init__(self, model_name: str = "openvla/openvla-7b", 
                 device: str = None, 
                 precision: str = "fp16"):
        """
        Initialize the VLA inference engine
        
        Args:
            model_name: Name of the VLA model to use
            device: Device to run inference on (e.g., 'cuda', 'cpu')
            precision: Model precision ("fp16", "fp32")
        """
        self.model_name = model_name
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.precision = precision
        self.model = None
        self.processor = None
        self.is_ready = False
        
        self._load_model()
    
    def _load_model(self) -> bool:
        """
        Load the VLA model for inference
        """
        try:
            torch_dtype = torch.float16 if self.precision == "fp16" else torch.float32
            
            print(f"Loading {self.model_name} for inference on {self.device}...")
            
            # Load the model and processor
            self.model = AutoModel.from_pretrained(
                self.model_name,
                torch_dtype=torch_dtype,
                low_cpu_mem_usage=True
            ).to(self.device)
            
            self.processor = AutoProcessor.from_pretrained(self.model_name)
            
            # Set model to evaluation mode
            self.model.eval()
            self.is_ready = True
            
            print(f"VLA model loaded successfully for inference")
            return True
            
        except Exception as e:
            print(f"Error loading VLA model: {e}")
            return False
    
    def preprocess_image(self, image: Union[str, Image.Image, np.ndarray]) -> Image.Image:
        """
        Preprocess the input image to the required format
        
        Args:
            image: Input image as path string, PIL Image, or numpy array
            
        Returns:
            Preprocessed PIL Image
        """
        if isinstance(image, str):
            # Load image from file path
            image = Image.open(image).convert("RGB")
        elif isinstance(image, np.ndarray):
            # Convert numpy array to PIL Image
            image = Image.fromarray(image)
        elif isinstance(image, Image.Image):
            # Already a PIL Image, ensure RGB format
            image = image.convert("RGB")
        else:
            raise ValueError(f"Unsupported image type: {type(image)}")
        
        return image
    
    def predict_action(self, image: Union[str, Image.Image, np.ndarray], 
                      instruction: str, 
                      confidence_threshold: float = 0.1) -> Optional[np.ndarray]:
        """
        Predict action from image and instruction
        
        Args:
            image: Input image (path, PIL Image, or numpy array)
            instruction: Natural language instruction
            confidence_threshold: Minimum confidence threshold for the prediction
            
        Returns:
            Action prediction as numpy array or None if prediction failed
        """
        if not self.is_ready:
            raise RuntimeError("VLA model is not ready. Check if it loaded correctly.")
        
        try:
            # Preprocess the image
            pil_image = self.preprocess_image(image)
            
            # Process the input with the processor
            inputs = self.processor(prompt=instruction, images=pil_image)
            
            # Move inputs to the appropriate device
            for key, value in inputs.items():
                if isinstance(value, torch.Tensor):
                    inputs[key] = value.to(self.device)
            
            # Perform inference
            with torch.no_grad():
                # Generate prediction
                outputs = self.model(**inputs)
                
                # Extract action logits
                action_logits = outputs.logits if hasattr(outputs, 'logits') else outputs
                
                # Compute action from logits (this is a simplified version)
                # In reality, the model output format may be different
                if isinstance(action_logits, torch.Tensor):
                    # Simple argmax approach - in practice, this would be more complex
                    action = torch.argmax(action_logits, dim=-1).cpu().numpy()
                    
                    # If action is multi-dimensional, flatten or process appropriately
                    if len(action.shape) > 1:
                        # Take the first sequence or flatten
                        action = action.flatten()[:7]  # Assuming 7-DOF action space
                else:
                    # Handle different output format
                    # This depends on the specific model architecture
                    print("Warning: Unexpected output format from model")
                    return None
            
            # Apply confidence threshold if applicable (simplified logic)
            # In a real implementation, this would be more sophisticated
            if hasattr(outputs, 'logits') and outputs.logits.numel() > 0:
                confidence = torch.softmax(outputs.logits, dim=-1).max().item()
                if confidence < confidence_threshold:
                    print(f"Warning: Low confidence prediction ({confidence:.3f} < threshold {confidence_threshold})")
            
            return action
            
        except Exception as e:
            print(f"Error during VLA inference: {e}")
            return None
    
    def predict_action_with_raw_output(self, image: Union[str, Image.Image, np.ndarray], 
                                      instruction: str) -> Tuple[Optional[np.ndarray], Any]:
        """
        Predict action and return both action and raw model output
        
        Args:
            image: Input image (path, PIL Image, or numpy array)
            instruction: Natural language instruction
            
        Returns:
            Tuple of (action prediction, raw model output)
        """
        if not self.is_ready:
            raise RuntimeError("VLA model is not ready. Check if it loaded correctly.")
        
        try:
            # Preprocess the image
            pil_image = self.preprocess_image(image)
            
            # Process the input with the processor
            inputs = self.processor(prompt=instruction, images=pil_image)
            
            # Move inputs to the appropriate device
            for key, value in inputs.items():
                if isinstance(value, torch.Tensor):
                    inputs[key] = value.to(self.device)
            
            # Perform inference
            with torch.no_grad():
                raw_output = self.model(**inputs)
                
                # Extract action from raw output
                if hasattr(raw_output, 'logits'):
                    action_logits = raw_output.logits
                    action = torch.argmax(action_logits, dim=-1).cpu().numpy()
                    
                    # Process action to appropriate shape
                    if len(action.shape) > 1:
                        action = action.flatten()[:7]  # Assuming 7-DOF action space
                else:
                    # Handle different output format
                    action = None
            
            return action, raw_output
            
        except Exception as e:
            print(f"Error during VLA inference: {e}")
            return None, None
    
    def batch_predict(self, images: List[Union[str, Image.Image, np.ndarray]], 
                     instructions: List[str]) -> List[Optional[np.ndarray]]:
        """
        Perform batch inference on multiple image-instruction pairs
        
        Args:
            images: List of input images
            instructions: List of corresponding instructions
            
        Returns:
            List of action predictions
        """
        if len(images) != len(instructions):
            raise ValueError("Number of images must match number of instructions")
        
        results = []
        for img, instr in zip(images, instructions):
            result = self.predict_action(img, instr)
            results.append(result)
        
        return results


def simple_inference_example():
    """
    Simple example of VLA inference
    """
    print("Running simple VLA inference example...")
    
    # Use a placeholder for the model to avoid large downloads in examples
    # In practice, this would use the actual OpenVLA model
    try:
        vla_infer = VLAInference(model_name="openvla/openvla-7b", device="cpu", precision="fp32")
        
        if vla_infer.is_ready:
            # Create a simple test image (in reality, this would be a real image)
            test_image = Image.new("RGB", (224, 224), color="blue")
            
            # Example instruction
            instruction = "Move the robot arm to the left"
            
            # Perform inference
            action = vla_infer.predict_action(test_image, instruction)
            
            if action is not None:
                print(f"Action prediction: {action}")
                print(f"Action shape: {action.shape if isinstance(action, np.ndarray) else 'N/A'}")
            else:
                print("No action prediction returned")
        else:
            print("VLA model not ready for inference")
            
    except Exception as e:
        print(f"Error in simple inference example: {e}")
        # Fallback: demonstrate the expected interface without actually running the model
        print("Expected interface demonstration:")
        print("- VLAInference class handles model loading")
        print("- predict_action() method takes image and instruction")
        print("- Returns action as numpy array")


def advanced_inference_example():
    """
    Advanced example with multiple predictions and raw output
    """
    print("\nRunning advanced VLA inference example...")
    
    try:
        vla_infer = VLAInference(model_name="openvla/openvla-7b", device="cpu", precision="fp32")
        
        if vla_infer.is_ready:
            # Test with multiple instructions
            test_image = Image.new("RGB", (224, 224), color="red")
            instructions = [
                "Pick up the object",
                "Move forward",
                "Turn right"
            ]
            
            for i, instr in enumerate(instructions):
                action, raw_output = vla_infer.predict_action_with_raw_output(test_image, instr)
                
                print(f"Instruction {i+1}: '{instr}'")
                print(f"Action: {action}")
                print(f"Raw output type: {type(raw_output)}")
                print("---")
        else:
            print("VLA model not ready for inference")
            
    except Exception as e:
        print(f"Error in advanced inference example: {e}")


def prepare_image_for_vla(image_path: str) -> Image.Image:
    """
    Utility function to prepare an image specifically for VLA models
    
    Args:
        image_path: Path to the input image
        
    Returns:
        Prepared PIL Image
    """
    try:
        image = Image.open(image_path).convert("RGB")
        
        # VLA models often expect images of a specific size
        # The exact size may vary depending on the model
        # Here we use a common size, but it should be confirmed for the specific model
        image = image.resize((224, 224))
        
        return image
    except Exception as e:
        print(f"Error preparing image {image_path}: {e}")
        return None


def validate_action_prediction(action: np.ndarray, min_length: int = 1, max_length: int = 24) -> bool:
    """
    Validate the action prediction
    
    Args:
        action: Action prediction array to validate
        min_length: Minimum expected length
        max_length: Maximum expected length
        
    Returns:
        True if valid, False otherwise
    """
    if action is None:
        return False
    
    if len(action) < min_length or len(action) > max_length:
        return False
    
    # Check if all values are finite
    if not np.all(np.isfinite(action)):
        return False
    
    return True


# Example usage
if __name__ == "__main__":
    print("Testing VLA Inference utilities...")
    
    # Run simple example
    simple_inference_example()
    
    # Run advanced example
    advanced_inference_example()
    
    # Example of validation
    test_action = np.random.rand(7)  # 7-DOF action
    is_valid = validate_action_prediction(test_action)
    print(f"\nAction validation result: {is_valid}")
    print(f"Test action: {test_action}")
    
    print("\nVLA Inference utilities test completed.")