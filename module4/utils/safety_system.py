"""
Safety system foundation for Module 4
Implements emergency protocols and safety checks for VLA systems
"""
import time
import logging
from typing import Dict, List, Optional, Any, Callable
from enum import Enum
from dataclasses import dataclass
import threading
import queue

logger = logging.getLogger(__name__)

class SafetyLevel(Enum):
    """
    Safety levels for different operational states
    """
    NORMAL = 1
    WARNING = 2
    EMERGENCY = 3

@dataclass
class SafetyEvent:
    """
    Data class for safety events
    """
    timestamp: float
    level: SafetyLevel
    source: str
    message: str
    action_taken: str
    context: Dict[str, Any]

class SafetySystem:
    """
    Main safety system that monitors and responds to safety events
    """
    
    def __init__(self, 
                 emergency_stop_callback: Optional[Callable[[], bool]] = None,
                 robot_name: str = "athena"):
        """
        Initialize safety system
        
        Args:
            emergency_stop_callback: Function to call for emergency stop
            robot_name: Name of the robot being protected
        """
        self.robot_name = robot_name
        self.emergency_stop_callback = emergency_stop_callback
        self.safety_level = SafetyLevel.NORMAL
        self.event_history = []
        self.safety_thresholds = self._initialize_safety_thresholds()
        
        # For monitoring and response
        self._monitoring = False
        self._monitoring_thread = None
        self._stop_event = threading.Event()
        self._action_queue = queue.Queue()
        
        # Register safety checks
        self._safety_checks = []
        
        logger.info(f"Safety system initialized for robot: {robot_name}")
    
    def _initialize_safety_thresholds(self) -> Dict[str, Any]:
        """
        Initialize safety thresholds and limits
        """
        return {
            # Joint limits
            'joint_position_limits': {
                'min': [-3.14] * 7,  # Example for 7-DOF robot
                'max': [3.14] * 7
            },
            
            # Velocity limits
            'max_joint_velocity': 2.0,  # rad/s
            'max_cartesian_velocity': 0.5,  # m/s
            
            # Force/torque limits
            'max_joint_torque': 100.0,  # N*m
            
            # Environmental limits
            'min_proximity_distance': 0.3,  # meters to obstacles
            'max_execution_time': 30.0,  # seconds for single action
            
            # System limits
            'max_cpu_utilization': 90.0,  # percent
            'max_memory_utilization': 90.0,  # percent
            'min_battery_level': 20.0,  # percent
        }
    
    def register_safety_check(self, check_func: Callable, check_name: str):
        """
        Register a safety check function
        
        Args:
            check_func: Function that takes action/state and returns (is_safe, reason)
            check_name: Name for the safety check
        """
        self._safety_checks.append({
            'func': check_func,
            'name': check_name
        })
        logger.info(f"Registered safety check: {check_name}")
    
    def validate_action(self, action: List[float], action_type: str = "joint_positions") -> Dict[str, Any]:
        """
        Validate an action against safety constraints
        
        Args:
            action: The action to validate
            action_type: Type of action ('joint_positions', 'joint_velocities', etc.)
            
        Returns:
            Dictionary with validation results
        """
        results = {
            'is_safe': True,
            'violations': [],
            'warnings': [],
            'action': action
        }
        
        # Apply registered safety checks
        for check in self._safety_checks:
            try:
                is_safe, reason = check['func'](action, action_type)
                if not is_safe:
                    results['is_safe'] = False
                    results['violations'].append({
                        'check': check['name'],
                        'reason': reason
                    })
            except Exception as e:
                logger.error(f"Safety check {check['name']} failed: {e}")
                results['is_safe'] = False
                results['violations'].append({
                    'check': check['name'],
                    'reason': f"Check failed: {str(e)}"
                })
        
        # If action is unsafe, log the violations
        if not results['is_safe']:
            logger.warning(f"Action validation failed: {results['violations']}")
        
        return results
    
    def check_joint_limits(self, joint_positions: List[float]) -> Dict[str, Any]:
        """
        Check if joint positions are within limits
        
        Args:
            joint_positions: List of joint positions to check
            
        Returns:
            Dictionary with check results
        """
        results = {
            'is_safe': True,
            'violations': [],
            'warnings': []
        }
        
        limits = self.safety_thresholds['joint_position_limits']
        
        for i, pos in enumerate(joint_positions):
            if pos < limits['min'][i]:
                results['is_safe'] = False
                results['violations'].append({
                    'joint': i,
                    'position': pos,
                    'limit': 'min',
                    'limit_value': limits['min'][i],
                    'message': f"Joint {i} position {pos} below minimum {limits['min'][i]}"
                })
            elif pos > limits['max'][i]:
                results['is_safe'] = False
                results['violations'].append({
                    'joint': i,
                    'position': pos,
                    'limit': 'max', 
                    'limit_value': limits['max'][i],
                    'message': f"Joint {i} position {pos} above maximum {limits['max'][i]}"
                })
        
        return results
    
    def check_velocity_limits(self, velocities: List[float], vel_type: str = "joint") -> Dict[str, Any]:
        """
        Check if velocities are within limits
        
        Args:
            velocities: List of velocities to check
            vel_type: Type of velocity ('joint' or 'cartesian')
            
        Returns:
            Dictionary with check results
        """
        results = {
            'is_safe': True,
            'violations': [],
            'warnings': []
        }
        
        if vel_type == "joint":
            max_vel = self.safety_thresholds['max_joint_velocity']
        elif vel_type == "cartesian":
            max_vel = self.safety_thresholds['max_cartesian_velocity']
        else:
            raise ValueError(f"Unknown velocity type: {vel_type}")
        
        for i, vel in enumerate(velocities):
            if abs(vel) > max_vel:
                results['is_safe'] = False
                results['violations'].append({
                    'index': i,
                    'velocity': vel,
                    'max_velocity': max_vel,
                    'message': f"Velocity {vel} exceeds maximum {max_vel}"
                })
        
        return results
    
    def check_execution_time(self, start_time: float) -> bool:
        """
        Check if execution is taking too long
        
        Args:
            start_time: When execution started
            
        Returns:
            True if still within time limit, False otherwise
        """
        elapsed = time.time() - start_time
        max_time = self.safety_thresholds['max_execution_time']
        return elapsed <= max_time
    
    def trigger_safety_event(self, level: SafetyLevel, source: str, message: str, action_taken: str = "None", context: Dict[str, Any] = None):
        """
        Trigger a safety event
        
        Args:
            level: Safety level of the event
            source: Source of the event
            message: Description of the event
            action_taken: Action taken in response
            context: Additional context information
        """
        event = SafetyEvent(
            timestamp=time.time(),
            level=level,
            source=source,
            message=message,
            action_taken=action_taken,
            context=context or {}
        )
        
        self.event_history.append(event)
        
        # Log the event
        logger.log(
            logging.ERROR if level == SafetyLevel.EMERGENCY else 
            logging.WARNING if level == SafetyLevel.WARNING else 
            logging.INFO,
            f"Safety Event [{level.name}]: {source} - {message} (Action: {action_taken})"
        )
        
        # Update safety level if this is a higher priority event
        if level.value > self.safety_level.value:
            self.safety_level = level
            
            # If emergency level, execute emergency protocols
            if level == SafetyLevel.EMERGENCY:
                self.execute_emergency_protocols()
    
    def execute_emergency_protocols(self):
        """
        Execute emergency safety protocols
        """
        logger.critical("Executing emergency safety protocols!")
        
        # First, try to stop robot motion
        if self.emergency_stop_callback:
            try:
                success = self.emergency_stop_callback()
                action_taken = f"Emergency stop: {'Success' if success else 'Failed'}"
            except Exception as e:
                logger.error(f"Emergency stop failed: {e}")
                action_taken = f"Emergency stop failed: {e}"
        else:
            action_taken = "No emergency stop callback defined"
        
        # Log emergency event
        self.trigger_safety_event(
            SafetyLevel.EMERGENCY,
            "SafetySystem",
            "Emergency protocols executed",
            action_taken
        )
    
    def start_monitoring(self):
        """
        Start safety monitoring thread
        """
        if self._monitoring:
            logger.warning("Safety monitoring already running")
            return
        
        self._monitoring = True
        self._stop_event.clear()
        self._monitoring_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self._monitoring_thread.start()
        
        logger.info("Safety monitoring started")
    
    def stop_monitoring(self):
        """
        Stop safety monitoring thread
        """
        if not self._monitoring:
            logger.warning("Safety monitoring not running")
            return
        
        self._monitoring = False
        self._stop_event.set()
        
        if self._monitoring_thread:
            self._monitoring_thread.join(timeout=2.0)
        
        logger.info("Safety monitoring stopped")
    
    def _monitoring_loop(self):
        """
        Internal monitoring loop that continuously checks for safety issues
        """
        while self._monitoring and not self._stop_event.is_set():
            try:
                # Check for queued actions to process
                try:
                    while True:
                        # Non-blocking get to process all queued items
                        safety_check = self._action_queue.get_nowait()
                        safety_check()
                except queue.Empty:
                    # No more items in queue
                    pass
                
                # Perform periodic system checks
                self._periodic_safety_checks()
                
                # Sleep before next iteration
                time.sleep(0.1)  # 10 Hz monitoring
                
            except Exception as e:
                logger.error(f"Error in safety monitoring loop: {e}")
                time.sleep(1.0)  # Wait before retrying
    
    def _periodic_safety_checks(self):
        """
        Perform periodic safety checks
        """
        # Check system resources (CPU, memory, etc.)
        # This would interface with system monitoring tools in a real implementation
        pass
    
    def get_safety_status(self) -> Dict[str, Any]:
        """
        Get current safety system status
        
        Returns:
            Dictionary with safety system status
        """
        return {
            'safety_level': self.safety_level.name,
            'total_events': len(self.event_history),
            'monitoring_active': self._monitoring,
            'recent_events': [
                {
                    'timestamp': e.timestamp,
                    'level': e.level.name,
                    'source': e.source,
                    'message': e.message,
                    'action_taken': e.action_taken
                }
                for e in self.event_history[-10:]  # Last 10 events
            ]
        }
    
    def reset_safety_level(self):
        """
        Reset safety level back to normal
        """
        logger.info("Resetting safety level to NORMAL")
        self.safety_level = SafetyLevel.NORMAL


class SafetySystemManager:
    """
    Manager class to coordinate safety systems across the VLA system
    """
    
    def __init__(self):
        self.safety_systems = {}
        self.global_safety_enabled = True
    
    def add_safety_system(self, name: str, safety_system: SafetySystem):
        """
        Add a safety system to be managed
        
        Args:
            name: Name for the safety system
            safety_system: SafetySystem instance
        """
        self.safety_systems[name] = safety_system
        logger.info(f"Added safety system: {name}")
    
    def validate_action_globally(self, action: List[float], action_type: str, source_system: str = "unknown") -> Dict[str, Any]:
        """
        Validate an action across all managed safety systems
        
        Args:
            action: Action to validate
            action_type: Type of action
            source_system: System requesting validation
            
        Returns:
            Dictionary with overall validation results
        """
        if not self.global_safety_enabled:
            return {
                'is_safe': True,
                'violations': [],
                'warnings': [],
                'action': action
            }
        
        overall_results = {
            'is_safe': True,
            'violations': [],
            'warnings': [],
            'action': action,
            'source_system': source_system
        }
        
        # Validate against each registered safety system
        for name, safety_system in self.safety_systems.items():
            try:
                result = safety_system.validate_action(action, action_type)
                
                if not result['is_safe']:
                    overall_results['is_safe'] = False
                    for violation in result['violations']:
                        overall_results['violations'].append({
                            'system': name,
                            'check': violation
                        })
                
                for warning in result['warnings']:
                    overall_results['warnings'].append({
                        'system': name, 
                        'check': warning
                    })
                    
            except Exception as e:
                logger.error(f"Error validating action with safety system {name}: {e}")
                overall_results['is_safe'] = False
                overall_results['violations'].append({
                    'system': name,
                    'check': {'error': str(e)}
                })
        
        # Log if action was rejected
        if not overall_results['is_safe']:
            logger.warning(f"Action rejected by safety systems: {overall_results['violations']}")
        
        return overall_results
    
    def trigger_global_event(self, level: SafetyLevel, source: str, message: str, action_taken: str = "None"):
        """
        Trigger a safety event across all managed systems
        """
        for name, safety_system in self.safety_systems.items():
            safety_system.trigger_safety_event(level, f"{source}[{name}]", message, action_taken)
    
    def get_all_safety_status(self) -> Dict[str, Any]:
        """
        Get safety status across all managed systems
        """
        status = {}
        for name, safety_system in self.safety_systems.items():
            status[name] = safety_system.get_safety_status()
        return status
    
    def enable_global_safety(self):
        """Enable global safety enforcement"""
        self.global_safety_enabled = True
        logger.info("Global safety enforcement enabled")
    
    def disable_global_safety(self):
        """Disable global safety enforcement"""
        self.global_safety_enabled = False
        logger.warning("Global safety enforcement disabled")


# Standalone function for easy usage
def initialize_safety_system(robot_name: str = "athena", emergency_stop_callback: Optional[Callable[[], bool]] = None) -> SafetySystem:
    """
    Initialize and return a safety system
    
    Args:
        robot_name: Name of the robot to protect
        emergency_stop_callback: Function to call for emergency stop
        
    Returns:
        Initialized SafetySystem instance
    """
    return SafetySystem(emergency_stop_callback, robot_name)


if __name__ == "__main__":
    # Example usage
    print("Testing Safety System...")
    
    # Initialize safety system (without emergency stop callback for simulation)
    safety_system = initialize_safety_system("athena")
    print("Safety System initialized successfully!")
    
    # Test action validation
    test_action = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
    result = safety_system.validate_action(test_action, "joint_positions")
    print(f"Action validation result: {result['is_safe']}")
    
    # Check joint limits
    limits_result = safety_system.check_joint_limits(test_action)
    print(f"Joint limits check: {limits_result['is_safe']}")
    
    # Trigger a safety event
    safety_system.trigger_safety_event(
        SafetyLevel.WARNING,
        "TestSource",
        "This is a test safety event",
        "No action taken"
    )
    
    # Get safety status
    status = safety_system.get_safety_status()
    print(f"Safety level: {status['safety_level']}")
    
    print("Safety System test completed.")