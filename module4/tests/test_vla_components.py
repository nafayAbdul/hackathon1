"""
Testing Infrastructure for VLA Components

This module provides testing infrastructure for VLA components.
"""

import unittest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the module4 directory to the path to import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from utils.vla_interface import VLAInterface, VLAConfig, VLAActionSpaceMapper
from utils.speech_processing import SpeechProcessor, NaturalLanguageProcessor, SpeechConfig
from utils.hardware_abstraction import HardwareManager, HardwareConfig
from utils.common_data_structures import (
    VisionInput, LanguageInput, ActionOutput, VLAPrediction, 
    RobotState, TaskPlan, PerceptionResult, 
    SafetyParameters, VLAConfig as VLAConfigStruct
)


class TestVLAInterface(unittest.TestCase):
    """
    Test cases for VLA Interface components
    """
    
    def setUp(self):
        """
        Set up test fixtures before each test method
        """
        self.config = VLAConfig(model_name="test_model", device="cpu")
        self.vla_interface = VLAInterface(self.config)
        
        # Mock the model loading since we don't have actual models in testing
        self.vla_interface.model = Mock()
        self.vla_interface.processor = Mock()
        self.vla_interface.is_initialized = True
    
    def test_vla_interface_initialization(self):
        """
        Test that VLAInterface initializes correctly with config
        """
        self.assertEqual(self.vla_interface.config.model_name, "test_model")
        self.assertEqual(self.vla_interface.config.device, "cpu")
    
    def test_predict_action_with_valid_inputs(self):
        """
        Test action prediction with valid image and instruction inputs
        """
        # Create mock inputs
        mock_image = np.random.rand(224, 224, 3)
        mock_instruction = "Pick up the red cup"
        
        # Mock the processor and model behavior
        mock_processed_inputs = {"input_ids": np.array([1, 2, 3]), "pixel_values": np.random.rand(1, 3, 224, 224)}
        self.vla_interface.processor.return_value = mock_processed_inputs
        
        mock_model_output = MagicMock()
        mock_model_output.logits = np.random.rand(1, 10)
        self.vla_interface.model.return_value = mock_model_output
        
        # Call the method
        result = self.vla_interface.predict_action(mock_image, mock_instruction)
        
        # Assertions
        self.assertIsInstance(result, np.ndarray)
        self.assertTrue(self.vla_interface.processor.called)
        self.assertTrue(self.vla_interface.model.called)
    
    def test_action_space_mapper_initialization(self):
        """
        Test that VLAActionSpaceMapper initializes for different robots
        """
        mapper = VLAActionSpaceMapper("athena")
        self.assertEqual(mapper.robot_type, "athena")
        self.assertIn("joint_names", mapper.robot_config)
    
    def test_athena_action_mapping(self):
        """
        Test that action mapping works for athena robot
        """
        mapper = VLAActionSpaceMapper("athena")
        mock_action = np.random.rand(6)  # 6 DOF action
        
        robot_action = mapper._map_athena_action(mock_action)
        
        self.assertEqual(len(robot_action), 24)  # Athena has 24 joints
        self.assertTrue(np.all(np.isfinite(robot_action)))
        
        # Test that actions are within joint limits
        joint_limits_min = np.array(mapper.robot_config["joint_limits"]["min"])
        joint_limits_max = np.array(mapper.robot_config["joint_limits"]["max"])
        
        self.assertTrue(np.all(robot_action >= joint_limits_min))
        self.assertTrue(np.all(robot_action <= joint_limits_max))


class TestSpeechProcessor(unittest.TestCase):
    """
    Test cases for Speech Processor components
    """
    
    def setUp(self):
        """
        Set up test fixtures before each test method
        """
        self.config = SpeechConfig(model_name="test_whisper", device="cpu")
        self.speech_processor = SpeechProcessor(self.config)
        
        # Mock the model loading
        self.speech_processor.model = Mock()
        self.speech_processor.is_initialized = True
    
    def test_speech_processor_initialization(self):
        """
        Test that SpeechProcessor initializes correctly with config
        """
        self.assertEqual(self.speech_processor.config.model_name, "test_whisper")
        self.assertEqual(self.speech_processor.config.device, "cpu")
    
    def test_transcribe_audio(self):
        """
        Test audio transcription functionality
        """
        # Create mock audio data
        mock_audio = np.random.rand(16000)  # 1 second of audio at 16kHz
        mock_result = Mock()
        mock_result.text = "This is a test"
        self.speech_processor.model.transcribe.return_value = {"text": "This is a test"}
        
        # Call the method
        result = self.speech_processor.transcribe_audio(mock_audio)
        
        # Assertions
        self.assertEqual(result, "This is a test")
        self.speech_processor.model.transcribe.assert_called_once()
    
    def test_natural_language_processor(self):
        """
        Test NLP component functionality
        """
        nlp_processor = NaturalLanguageProcessor("test_model")
        
        # Mock model components
        nlp_processor.model = Mock()
        nlp_processor.tokenizer = Mock()
        nlp_processor.is_initialized = True
        
        # Mock tokenizer behavior
        nlp_processor.tokenizer.encode.return_value = np.array([[1, 2, 3]])
        nlp_processor.tokenizer.decode.return_value = '{"action": "pick", "object": "cup", "target_location": "table", "secondary_objects": [], "intent": "Pick up the cup"}'
        nlp_processor.tokenizer.pad_token = "pad"
        
        # Test command parsing
        command = "Pick up the red cup from the table"
        parsed = nlp_processor.parse_command(command)
        
        self.assertEqual(parsed["action"], "pick")
        self.assertEqual(parsed["object"], "cup")
        self.assertEqual(parsed["target_location"], "table")


class TestHardwareAbstraction(unittest.TestCase):
    """
    Test cases for Hardware Abstraction Layer
    """
    
    def test_hardware_manager_initialization(self):
        """
        Test that HardwareManager initializes correctly
        """
        config = HardwareConfig(platform="simulation", robot_type="athena")
        manager = HardwareManager(config)
        
        self.assertEqual(manager.config.platform, "simulation")
        self.assertEqual(manager.config.robot_type, "athena")
        self.assertTrue(hasattr(manager, 'hardware_interface'))
    
    def test_simulation_interface_connection(self):
        """
        Test that simulation interface connects successfully
        """
        config = HardwareConfig(platform="simulation", robot_type="athena")
        manager = HardwareManager(config)
        
        success = manager.connect()
        self.assertTrue(success)
        self.assertTrue(manager.active)
    
    def test_robot_state_retrieval(self):
        """
        Test that robot state can be retrieved
        """
        config = HardwareConfig(platform="simulation", robot_type="athena")
        manager = HardwareManager(config)
        manager.connect()
        
        state = manager.get_robot_state()
        
        self.assertIsInstance(state, RobotState)
        self.assertIsInstance(state.joint_state.positions, list)
        self.assertEqual(len(state.joint_state.positions), 24)  # Athena has 24 joints


class TestCommonDataStructures(unittest.TestCase):
    """
    Test cases for common data structures
    """
    
    def test_vision_input_creation(self):
        """
        Test VisionInput data structure
        """
        image = np.random.rand(224, 224, 3)
        vision_input = VisionInput(
            image=image,
            camera_intrinsics=np.eye(3),
            objects=[{"name": "cup", "bbox": [0, 0, 100, 100]}]
        )
        
        self.assertEqual(vision_input.image.shape, (224, 224, 3))
        self.assertIsNotNone(vision_input.camera_intrinsics)
        self.assertEqual(len(vision_input.objects), 1)
    
    def test_language_input_creation(self):
        """
        Test LanguageInput data structure
        """
        language_input = LanguageInput(
            text="Pick up the red cup",
            intent="grasping",
            entities={"object": "cup", "color": "red"}
        )
        
        self.assertEqual(language_input.text, "Pick up the red cup")
        self.assertEqual(language_input.intent, "grasping")
        self.assertEqual(language_input.entities["object"], "cup")
    
    def test_action_output_creation(self):
        """
        Test ActionOutput data structure
        """
        action_output = ActionOutput(
            joint_positions=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            cartesian_pose=(0.5, 0.5, 0.3, 0, 0, 0)
        )
        
        self.assertEqual(len(action_output.joint_positions), 7)
        self.assertEqual(action_output.cartesian_pose, (0.5, 0.5, 0.3, 0, 0, 0))
    
    def test_vla_prediction_validation(self):
        """
        Test VLAPrediction validation
        """
        # Create all required components
        image = np.random.rand(224, 224, 3)
        vision_input = VisionInput(image=image)
        
        language_input = LanguageInput(text="Test command")
        
        action_output = ActionOutput(joint_positions=[0.1, 0.2, 0.3])
        
        vla_prediction = VLAPrediction(
            vision_input=vision_input,
            language_input=language_input,
            action_output=action_output
        )
        
        # Validate the prediction
        from utils.common_data_structures import validate_vla_prediction
        is_valid = validate_vla_prediction(vla_prediction)
        
        self.assertTrue(is_valid)


def run_tests():
    """
    Run all tests
    """
    # Create a test suite
    test_suite = unittest.TestSuite()
    
    # Add all test cases
    test_suite.addTest(unittest.makeSuite(TestVLAInterface))
    test_suite.addTest(unittest.makeSuite(TestSpeechProcessor))
    test_suite.addTest(unittest.makeSuite(TestHardwareAbstraction))
    test_suite.addTest(unittest.makeSuite(TestCommonDataStructures))
    
    # Run the tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    return result


if __name__ == '__main__':
    run_tests()