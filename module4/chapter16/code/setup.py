"""
OpenVLA Setup and Initialization Utilities
Listing 16.1: OpenVLA setup and initialization utilities
"""
import os
import torch
from transformers import AutoModel, AutoProcessor
from typing import Optional, Dict, Any


class OpenVLASetup:
    """
    Class to handle OpenVLA model setup and initialization
    """
    
    def __init__(self, model_name: str = "openvla/openvla-7b", device: str = None):
        """
        Initialize the OpenVLA setup
        
        Args:
            model_name: Name of the OpenVLA model to use
            device: Device to load the model on (e.g., 'cuda', 'cpu')
        """
        self.model_name = model_name
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model = None
        self.processor = None
        self.is_loaded = False
        
        print(f"OpenVLASetup initialized with model: {model_name} on device: {self.device}")
    
    def download_model(self, local_dir: Optional[str] = None) -> str:
        """
        Download the OpenVLA model if not already present
        
        Args:
            local_dir: Local directory to download the model to
            
        Returns:
            Path to the downloaded model
        """
        try:
            from huggingface_hub import snapshot_download
            
            if local_dir is None:
                local_dir = f"./models/{self.model_name.replace('/', '_')}"
            
            print(f"Downloading {self.model_name} to {local_dir}...")
            
            # Create directory if it doesn't exist
            os.makedirs(local_dir, exist_ok=True)
            
            # Download the model from Hugging Face
            snapshot_download(
                repo_id=self.model_name,
                local_dir=local_dir,
                local_dir_use_symlinks=False
            )
            
            print(f"Model downloaded successfully to {local_dir}")
            return local_dir
            
        except ImportError:
            print("huggingface_hub not installed. Install with: pip install huggingface_hub")
            return None
        except Exception as e:
            print(f"Error downloading model: {e}")
            return None
    
    def load_model(self, model_path: Optional[str] = None, precision: str = "fp16") -> bool:
        """
        Load the OpenVLA model
        
        Args:
            model_path: Path to the model (if different from HuggingFace)
            precision: Precision for the model ("fp16", "fp32", "int8")
            
        Returns:
            True if model loaded successfully, False otherwise
        """
        try:
            print(f"Loading OpenVLA model: {self.model_name}")
            
            # Determine precision for loading
            torch_dtype = torch.float16 if precision == "fp16" else torch.float32
            
            # Load the model and processor
            self.model = AutoModel.from_pretrained(
                model_path or self.model_name,
                torch_dtype=torch_dtype,
                low_cpu_mem_usage=True
            ).to(self.device)
            
            self.processor = AutoProcessor.from_pretrained(model_path or self.model_name)
            
            # Set model to evaluation mode
            self.model.eval()
            
            self.is_loaded = True
            print(f"Model loaded successfully on {self.device} with {precision} precision")
            return True
            
        except Exception as e:
            print(f"Error loading OpenVLA model: {e}")
            return False
    
    def verify_model_loaded(self) -> bool:
        """
        Verify that the model has been properly loaded
        
        Returns:
            True if model is loaded, False otherwise
        """
        return self.is_loaded and self.model is not None and self.processor is not None
    
    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the loaded model
        
        Returns:
            Dictionary with model information
        """
        if not self.verify_model_loaded():
            return {"error": "Model not loaded"}
        
        return {
            "model_name": self.model_name,
            "device": self.device,
            "precision": str(next(self.model.parameters()).dtype),
            "num_parameters": sum(p.numel() for p in self.model.parameters()),
            "loaded": self.is_loaded
        }


def initialize_openvla(model_name: str = "openvla/openvla-7b", 
                      device: str = None, 
                      precision: str = "fp16") -> Optional[OpenVLASetup]:
    """
    Initialize OpenVLA model with default parameters
    
    Args:
        model_name: Name of the OpenVLA model to use
        device: Device to load the model on
        precision: Precision for the model ("fp16", "fp32", "int8")
        
    Returns:
        OpenVLASetup instance if successful, None otherwise
    """
    setup = OpenVLASetup(model_name, device)
    
    # Attempt to load the model directly from Hugging Face
    success = setup.load_model(precision=precision)
    
    if success:
        return setup
    else:
        print("Failed to initialize OpenVLA. Try downloading the model first with download_model().")
        return None


def quick_start(model_name: str = "openvla/openvla-7b", 
                device: str = None,
                precision: str = "fp16",
                download_if_missing: bool = True) -> Optional[OpenVLASetup]:
    """
    Quick start function to initialize OpenVLA with minimal setup
    
    Args:
        model_name: Name of the OpenVLA model to use
        device: Device to load the model on
        precision: Precision for the model ("fp16", "fp32", "int8")
        download_if_missing: Whether to download the model if not found locally
        
    Returns:
        OpenVLASetup instance if successful, None otherwise
    """
    print("Starting OpenVLA quick initialization...")
    
    # Initialize setup
    setup = OpenVLASetup(model_name, device)
    
    # Try to load the model
    success = setup.load_model(precision=precision)
    
    # If loading failed and download_if_missing is True, try to download first
    if not success and download_if_missing:
        print("Model not found locally. Attempting to download...")
        model_path = setup.download_model()
        if model_path:
            success = setup.load_model(model_path=model_path, precision=precision)
    
    return setup if success else None


# Example usage and testing
if __name__ == "__main__":
    print("Testing OpenVLA setup utilities...")
    
    # Example 1: Standard initialization
    print("\n1. Standard initialization:")
    vla_setup = initialize_openvla()
    if vla_setup and vla_setup.verify_model_loaded():
        print("✓ Standard initialization successful")
        print(f"Model info: {vla_setup.get_model_info()}")
    else:
        print("✗ Standard initialization failed (expected if model not available)")
    
    # Example 2: Quick start with fallback
    print("\n2. Quick start with download fallback:")
    vla_quick = quick_start(download_if_missing=False)  # Set to False to avoid large download
    if vla_quick and vla_quick.verify_model_loaded():
        print("✓ Quick start successful")
    else:
        print("✗ Quick start failed (expected if model not available)")
    
    print("\nOpenVLA setup utilities test completed.")