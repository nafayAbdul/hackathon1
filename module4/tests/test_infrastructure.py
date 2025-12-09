"""
Testing infrastructure for VLA components in Module 4
Provides testing utilities and common test scenarios for VLA systems
"""
import unittest
import tempfile
import os
from unittest.mock import Mock, MagicMock, patch
import numpy as np
import torch
from PIL import Image
from typing import Dict, List, Any, Optional
import logging
import time

from module4.utils.vla_interface import VLAInterface, initialize_vla_interface
from module4.utils.speech_processing import SpeechProcessingUtilities, initialize_speech_processor
from module4.utils.hardware_abstraction import HardwareAbstractionLayer, initialize_hardware_abstraction
from module4.utils.safety_system import SafetySystem, SafetySystemManager, SafetyLevel, initialize_safety_system
from module4.utils.config import ConfigManager, get_current_vla_config

logger = logging.getLogger(__name__)

class MockVLAInterface:
    """
    Mock implementation of VLA Interface for testing
    """
    def __init__(self, *args, **kwargs):
        self.model_name = kwargs.get('model_name', 'test-model')
        self.device = kwargs.get('device', 'cpu')
    
    def predict_action(self, image, instruction: str, **kwargs):
        """
        Mock implementation of action prediction
        """
        # Return a fixed action for testing purposes
        return {
            'raw_action': f"test_action_for_{instruction}",
            'robot_action': [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            'success': True,
            'message': 'Mock action predicted successfully'
        }

class MockSpeechProcessor:
    """
    Mock implementation of Speech Processing Utilities for testing
    """
    def __init__(self, *args, **kwargs):
        self.device = kwargs.get('device', 'cpu')
        self.language = kwargs.get('language', 'en')
    
    def transcribe_audio(self, audio_file: str):
        """
        Mock implementation of audio transcription
        """
        # Return a fixed transcription for testing
        return {
            'text': 'Test voice command for robot',
            'language': self.language,
            'duration': 2.5,
            'success': True,
            'message': 'Mock transcription completed successfully'
        }
    
    def process_voice_command(self, audio_input):
        """
        Mock implementation of voice command processing
        """
        transcription_result = self.transcribe_audio(audio_input) if isinstance(audio_input, str) else {
            'text': 'Test voice command for robot',
            'language': self.language,
            'success': True,
            'message': 'Mock command processed successfully'
        }
        
        if transcription_result['success']:
            return {
                'command': {'raw_command': transcription_result['text']},
                'raw_transcription': transcription_result['text'],
                'language': transcription_result['language'],
                'success': True,
                'message': 'Mock command processed successfully'
            }
        else:
            return {
                'command': {},
                'raw_transcription': '',
                'success': False,
                'message': transcription_result['message']
            }

class MockRobotInterface:
    """
    Mock robot interface for testing without actual hardware
    """
    def __init__(self, robot_name: str = "test_robot", use_real_hardware: bool = False):
        self.robot_name = robot_name
        self.use_real_hardware = use_real_hardware
        self._connected = False
        self._joint_states = [0.0] * 7
        self._action_history = []
    
    def connect(self):
        self._connected = True
        return True
    
    def disconnect(self):
        self._connected = False
        return True
    
    def execute_action(self, action: List[float], action_type: str = "joint_positions"):
        if not self._connected:
            return False
        
        # Record the action
        self._action_history.append({
            'action': action.copy(),
            'type': action_type,
            'timestamp': time.time()
        })
        
        # Update joint states based on action
        if action_type == "joint_positions":
            self._joint_states = action.copy()
        elif action_type == "joint_velocities":
            dt = 0.1  # Simulated time step
            for i in range(len(self._joint_states)):
                self._joint_states[i] += action[i] * dt
        
        return True
    
    def get_joint_states(self):
        return self._joint_states.copy()
    
    def get_robot_pose(self):
        # Return a mock pose
        from geometry_msgs.msg import Pose
        return Pose()
    
    def is_connected(self):
        return self._connected


class TestVLAInterface(unittest.TestCase):
    """
    Tests for VLA Interface
    """
    
    def setUp(self):
        # Use mock interface for testing to avoid loading large models
        with patch('module4.utils.vla_interface.VLAInterface', MockVLAInterface):
            self.vla_interface = initialize_vla_interface()
    
    def test_predict_action(self):
        """
        Test that action prediction works correctly
        """
        # Create a mock image
        mock_image = Image.new('RGB', (224, 224))
        
        # Test action prediction
        result = self.vla_interface.predict_action(mock_image, "Test instruction")
        
        # Verify result structure
        self.assertTrue(result['success'])
        self.assertIsNotNone(result['raw_action'])
        self.assertIsNotNone(result['robot_action'])
        self.assertIsInstance(result['robot_action'], list)
        self.assertEqual(len(result['robot_action']), 7)  # 7-DOF robot
    
    def test_batch_predict(self):
        """
        Test batch prediction functionality
        """
        # Create mock images
        mock_images = [Image.new('RGB', (224, 224)) for _ in range(2)]
        instructions = ["Instruction 1", "Instruction 2"]
        
        # Since we're using a mock, batch_predict uses predict_action internally
        # so this test verifies the interface can handle multiple requests
        results = self.vla_interface.batch_predict(mock_images, instructions)
        
        self.assertEqual(len(results), 2)
        for result in results:
            self.assertTrue(result['success'])


class TestSpeechProcessing(unittest.TestCase):
    """
    Tests for Speech Processing Utilities
    """
    
    def setUp(self):
        # Use mock processor for testing to avoid loading large models
        with patch('module4.utils.speech_processing.SpeechProcessingUtilities', MockSpeechProcessor):
            self.speech_processor = initialize_speech_processor()
    
    def test_transcribe_audio(self):
        """
        Test audio transcription functionality
        """
        # Create a temporary audio file for testing
        with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as tmp_file:
            tmp_path = tmp_file.name
        
        try:
            # Test transcription
            result = self.speech_processor.transcribe_audio(tmp_path)
            
            self.assertTrue(result['success'])
            self.assertIn('Test voice command', result['text'])
        finally:
            # Clean up temp file
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
    
    def test_process_voice_command(self):
        """
        Test voice command processing
        """
        # Create a temporary audio file for testing
        with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as tmp_file:
            tmp_path = tmp_file.name
        
        try:
            # Test command processing
            result = self.speech_processor.process_voice_command(tmp_path)
            
            self.assertTrue(result['success'])
            self.assertIsNotNone(result['command'])
        finally:
            # Clean up temp file
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)


class TestHardwareAbstraction(unittest.TestCase):
    """
    Tests for Hardware Abstraction Layer
    """
    
    def setUp(self):
        # Mock the robot interface to avoid needing real hardware
        with patch('module4.utils.hardware_abstraction.RealRobotInterface', MockRobotInterface), \
             patch('module4.utils.hardware_abstraction.SimulatedRobotInterface', MockRobotInterface):
            self.hal = initialize_hardware_abstraction("test_robot", use_real_hardware=False)
    
    def test_connect_disconnect(self):
        """
        Test robot connection and disconnection
        """
        # Test connection
        connected = self.hal.connect()
        self.assertTrue(connected)
        self.assertTrue(self.hal.is_connected())
        
        # Test disconnection
        disconnected = self.hal.disconnect()
        self.assertTrue(disconnected)
        self.assertFalse(self.hal.is_connected())
    
    def test_execute_action(self):
        """
        Test action execution
        """
        # Connect first
        self.hal.connect()
        
        # Test action execution
        test_action = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        success = self.hal.execute_action(test_action, "joint_positions")
        
        self.assertTrue(success)
        
        # Verify joint states were updated
        joint_states = self.hal.get_joint_states()
        self.assertEqual(joint_states, test_action)
        
        # Disconnect
        self.hal.disconnect()
    
    def test_get_robot_info(self):
        """
        Test getting robot information
        """
        info = self.hal.get_robot_info()
        
        self.assertEqual(info['robot_name'], 'test_robot')
        self.assertFalse(info['use_real_hardware'])


class TestSafetySystem(unittest.TestCase):
    """
    Tests for Safety System
    """
    
    def setUp(self):
        # Mock emergency stop callback
        mock_callback = Mock(return_value=True)
        self.safety_system = initialize_safety_system("test_robot", mock_callback)
    
    def test_safety_system_initialization(self):
        """
        Test safety system initialization
        """
        self.assertEqual(self.safety_system.robot_name, "test_robot")
        self.assertEqual(self.safety_system.safety_level, SafetyLevel.NORMAL)
    
    def test_safety_validation(self):
        """
        Test safety validation functionality
        """
        # Test valid action
        valid_action = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        result = self.safety_system.validate_action(valid_action, "joint_positions")
        
        self.assertTrue(result['is_safe'])
        self.assertEqual(len(result['violations']), 0)
    
    def test_joint_limits_check(self):
        """
        Test joint limits checking
        """
        # Test within limits
        safe_joints = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        result = self.safety_system.check_joint_limits(safe_joints)
        
        self.assertTrue(result['is_safe'])
        self.assertEqual(len(result['violations']), 0)
        
        # Test outside limits
        unsafe_joints = [5.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]  # First joint exceeds limit
        result = self.safety_system.check_joint_limits(unsafe_joints)
        
        self.assertFalse(result['is_safe'])
        self.assertGreater(len(result['violations']), 0)
    
    def test_safety_event_triggering(self):
        """
        Test safety event triggering
        """
        initial_event_count = len(self.safety_system.event_history)
        
        # Trigger a warning event
        self.safety_system.trigger_safety_event(
            SafetyLevel.WARNING,
            "TestSource",
            "Test warning message"
        )
        
        # Check that event was recorded
        self.assertEqual(len(self.safety_system.event_history), initial_event_count + 1)
        latest_event = self.safety_system.event_history[-1]
        
        self.assertEqual(latest_event.level, SafetyLevel.WARNING)
        self.assertEqual(latest_event.source, "TestSource")
        self.assertEqual(latest_event.message, "Test warning message")


class TestConfigManager(unittest.TestCase):
    """
    Tests for Configuration Manager
    """
    
    def setUp(self):
        self.config_mgr = ConfigManager(default_tier=1)
    
    def test_config_initialization(self):
        """
        Test configuration manager initialization
        """
        self.assertEqual(self.config_mgr.default_tier, 1)
        self.assertEqual(self.config_mgr.current_tier, 1)
        
        # Check that config has required fields
        config = self.config_mgr.get_current_config()
        self.assertIn('current_tier', config)
        self.assertIn('model_precision', config)
        self.assertIn('batch_size', config)
    
    def test_tier_update(self):
        """
        Test updating hardware tier
        """
        # Change to tier 2 (Jetson Orin NX)
        success = self.config_mgr.update_tier(2)
        self.assertTrue(success)
        self.assertEqual(self.config_mgr.current_tier, 2)
        
        # Verify config changed appropriately
        config = self.config_mgr.get_current_config()
        self.assertEqual(config['current_tier'], 2)
        
        # Verify VLA config reflects the tier
        vla_config = self.config_mgr.get_vla_pipeline_config()
        self.assertGreaterEqual(vla_config['max_inference_latency'], 200.0)  # Tier 2 should have higher latency
    
    def test_performance_parameters(self):
        """
        Test getting performance parameters for different tiers
        """
        # Get parameters for Tier 1 (RTX 4090)
        tier1_latency = self.config_mgr.get_performance_parameter('max_inference_latency_ms', 1)
        
        # Get parameters for Tier 2 (Jetson Orin NX)
        tier2_latency = self.config_mgr.get_performance_parameter('max_inference_latency_ms', 2)
        
        # Tier 2 should have higher latency than Tier 1
        self.assertGreater(tier2_latency, tier1_latency)


class IntegrationTest(unittest.TestCase):
    """
    Integration tests that verify components work together
    """
    
    def setUp(self):
        # Use mocks for components that require external resources
        self.mock_vla = MockVLAInterface()
        self.mock_speech = MockSpeechProcessor()
        self.mock_robot = MockRobotInterface()
        self.mock_safety = Mock(spec=SafetySystem)
        self.mock_safety.validate_action.return_value = {'is_safe': True, 'violations': [], 'warnings': []}
        
        # Set up safety system manager with mock
        self.safety_manager = Mock(spec=SafetySystemManager)
        self.safety_manager.validate_action_globally.return_value = {
            'is_safe': True, 
            'violations': [], 
            'warnings': []
        }
    
    def test_vla_speech_integration(self):
        """
        Test integration between VLA and speech processing
        """
        # Simulate speech command processing (mocked)
        speech_result = self.mock_speech.process_voice_command("test_audio.wav")
        
        # Verify we got a command
        self.assertTrue(speech_result['success'])
        
        # Simulate using that command with VLA (mocked)
        mock_image = Image.new('RGB', (224, 224))
        vla_result = self.mock_vla.predict_action(mock_image, speech_result['command']['raw_command'])
        
        # Verify VLA processed the command
        self.assertTrue(vla_result['success'])
        self.assertIsNotNone(vla_result['robot_action'])
    
    def test_complete_pipeline_with_safety(self):
        """
        Test complete pipeline with safety validation
        """
        # Get an action from VLA
        mock_image = Image.new('RGB', (224, 224))
        vla_result = self.mock_vla.predict_action(mock_image, "Move to target location")
        
        self.assertTrue(vla_result['success'])
        
        # Validate the action with safety system
        safety_validation = self.safety_manager.validate_action_globally(
            vla_result['robot_action'], 
            "joint_positions",
            "VLA_Interface"
        )
        
        # Verify action passed safety validation
        self.assertTrue(safety_validation['is_safe'])
        
        # Execute on mock robot
        action_success = self.mock_robot.execute_action(vla_result['robot_action'], "joint_positions")
        self.assertTrue(action_success)


def run_all_tests():
    """
    Convenience function to run all tests
    """
    print("Running VLA Component Tests...")
    
    # Create test suite
    test_suite = unittest.TestSuite()
    
    # Add all test cases
    test_suite.addTest(unittest.makeSuite(TestVLAInterface))
    test_suite.addTest(unittest.makeSuite(TestSpeechProcessing))
    test_suite.addTest(unittest.makeSuite(TestHardwareAbstraction))
    test_suite.addTest(unittest.makeSuite(TestSafetySystem))
    test_suite.addTest(unittest.makeSuite(TestConfigManager))
    test_suite.addTest(unittest.makeSuite(IntegrationTest))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Print summary
    print(f"\nTests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success rate: {(result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100:.1f}%")
    
    return result


def create_test_report():
    """
    Create a test report in a temporary directory
    """
    import datetime
    
    report_dir = tempfile.mkdtemp(prefix="vla_test_report_")
    report_path = os.path.join(report_dir, "test_report.txt")
    
    with open(report_path, 'w') as f:
        f.write("VLA Component Test Report\n")
        f.write(f"Generated: {datetime.datetime.now().isoformat()}\n")
        f.write("="*50 + "\n\n")
        
        # Run tests and capture output
        import io
        from contextlib import redirect_stdout
        
        output_buffer = io.StringIO()
        with redirect_stdout(output_buffer):
            result = run_all_tests()
        
        f.write(output_buffer.getvalue())
        
        f.write(f"\nTest Report saved to: {report_path}")
    
    return report_path


if __name__ == "__main__":
    print("Testing Infrastructure for VLA Components")
    print("==========================================")
    
    # Run all tests
    result = run_all_tests()
    
    # Create a test report
    report_path = create_test_report()
    print(f"\nTest report created at: {report_path}")
    
    print("\nTesting Infrastructure execution completed.")