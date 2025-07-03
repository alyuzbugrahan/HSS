#!/usr/bin/env python3
"""
Example: Coordinate System Usage for Hybrid Mode
Demonstrates how to use the coordinate transformer for both static and moving targets
"""

import sys
import time
import threading
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent.parent / "src"))

from motors.coordinate_transformer import (
    CoordinateTransformer, 
    DetectedTarget, 
    TargetType
)
from motors.stepper_controller import DualMotorSystem

class HybridTargetingSystem:
    """
    Example implementation of hybrid targeting system
    Combines coordinate transformation with motor control
    """
    
    def __init__(self):
        """Initialize hybrid targeting system"""
        self.transformer = CoordinateTransformer("config/camera_config.json")
        self.motor_system = DualMotorSystem("config/motor_config.json")
        
        # System state
        self.is_running = False
        self.static_sequence_active = False
        self.moving_target_thread = None
        
        print("Hybrid targeting system initialized")
    
    def start_system(self):
        """Start the targeting system"""
        print("Starting hybrid targeting system...")
        
        # Home all motors first
        if not self.motor_system.home_all_motors():
            print("ERROR: Motor homing failed!")
            return False
        
        self.is_running = True
        
        # Start moving target monitoring thread
        self.moving_target_thread = threading.Thread(target=self._monitor_moving_targets)
        self.moving_target_thread.daemon = True
        self.moving_target_thread.start()
        
        print("System started successfully!")
        return True
    
    def process_frame_detections(self, mock_detections):
        """
        Process YOLO detections from a single frame
        
        Args:
            mock_detections: List of DetectedTarget objects (normally from YOLO)
        """
        print(f"\nProcessing {len(mock_detections)} detections...")
        
        # Separate static and moving targets
        static_targets, moving_targets = self.transformer.process_detections(mock_detections)
        
        print(f"Found {len(static_targets)} static, {len(moving_targets)} moving targets")
        
        # Handle static targets (if no sequence is currently running)
        if static_targets and not self.static_sequence_active:
            self._execute_static_sequence(static_targets)
        
        # Handle moving targets (always update)
        if moving_targets:
            self._update_moving_targets(moving_targets)
    
    def _execute_static_sequence(self, static_targets):
        """Execute static target shooting sequence"""
        self.static_sequence_active = True
        
        try:
            # Plan optimal shooting sequence
            sequence = self.transformer.plan_static_shooting_sequence(static_targets)
            
            print(f"Executing static sequence: {len(sequence)} targets")
            
            for i, target in enumerate(sequence):
                print(f"  Target {i+1}/{len(sequence)}: "
                      f"({target.azimuth:.1f}°, {target.elevation:.1f}°) "
                      f"Priority: {target.priority}")
                
                # Move to target
                success = self.motor_system.move_to_target(
                    target.azimuth, 
                    target.elevation
                )
                
                if success:
                    # Simulate shooting
                    print(f"    SHOT FIRED at target {i+1}")
                    time.sleep(0.5)  # Stabilization delay
                else:
                    print(f"    FAILED to reach target {i+1}")
            
            # Return to home
            print("Static sequence complete, returning home...")
            self.motor_system.move_to_target(0.0, 0.0)
            
        finally:
            self.static_sequence_active = False
    
    def _update_moving_targets(self, moving_targets):
        """Update current moving targets list"""
        # In real implementation, this would update a shared data structure
        # that the moving target monitoring thread reads from
        print(f"Updated moving targets: {len(moving_targets)} active")
        
        for target in moving_targets:
            print(f"  Moving target: ({target.azimuth:.1f}°, {target.elevation:.1f}°) "
                  f"Priority: {target.priority}")
    
    def _monitor_moving_targets(self):
        """Monitor and engage moving targets (runs in separate thread)"""
        print("Moving target monitoring started...")
        
        while self.is_running:
            time.sleep(0.1)  # 10Hz monitoring rate
            
            # In real implementation, this would:
            # 1. Get latest moving targets from shared data structure
            # 2. Select best target
            # 3. Engage if no static sequence is running
            
            # For demo, we just sleep
            pass
    
    def manual_target_test(self, pixel_x, pixel_y):
        """
        Manual test: click a pixel coordinate and move motors there
        
        Args:
            pixel_x, pixel_y: Pixel coordinates to target
        """
        print(f"\nManual target test: Pixel ({pixel_x}, {pixel_y})")
        
        # Convert to motor angles
        azimuth, elevation = self.transformer.calibration.pixel_to_angle(pixel_x, pixel_y)
        
        print(f"Converted to motor angles: ({azimuth:.2f}°, {elevation:.2f}°)")
        
        # Move to target
        success = self.motor_system.move_to_target(azimuth, elevation)
        
        if success:
            print("Manual targeting successful!")
        else:
            print("Manual targeting failed!")
        
        return success
    
    def stop_system(self):
        """Stop the targeting system"""
        print("Stopping hybrid targeting system...")
        
        self.is_running = False
        
        # Emergency stop all motors
        self.motor_system.emergency_stop_all()
        
        # Wait for moving target thread to finish
        if self.moving_target_thread:
            self.moving_target_thread.join(timeout=1.0)
        
        # Cleanup resources
        self.motor_system.cleanup()
        
        print("System stopped.")


def create_mock_detections():
    """Create mock YOLO detections for testing"""
    current_time = time.time()
    
    # Static targets
    static_targets = [
        DetectedTarget(
            pixel_x=300, pixel_y=200, confidence=0.9, 
            class_name='red_balloon', bbox=(280, 180, 320, 220),
            target_type=TargetType.STATIC, timestamp=current_time, track_id=1
        ),
        DetectedTarget(
            pixel_x=500, pixel_y=300, confidence=0.8,
            class_name='red_balloon', bbox=(480, 280, 520, 320),
            target_type=TargetType.STATIC, timestamp=current_time, track_id=2
        ),
        DetectedTarget(
            pixel_x=100, pixel_y=150, confidence=0.85,
            class_name='red_balloon', bbox=(80, 130, 120, 170),
            target_type=TargetType.STATIC, timestamp=current_time, track_id=3
        )
    ]
    
    # Moving targets
    moving_targets = [
        DetectedTarget(
            pixel_x=400, pixel_y=250, confidence=0.75,
            class_name='red_balloon', bbox=(380, 230, 420, 270),
            target_type=TargetType.MOVING, timestamp=current_time, track_id=4
        )
    ]
    
    # Friendly targets (should be ignored)
    friendly_targets = [
        DetectedTarget(
            pixel_x=200, pixel_y=100, confidence=0.9,
            class_name='blue_balloon', bbox=(180, 80, 220, 120),
            target_type=TargetType.STATIC, timestamp=current_time, track_id=5
        )
    ]
    
    return static_targets + moving_targets + friendly_targets


def main():
    """Main demo function"""
    print("=== Hybrid Targeting System Demo ===\n")
    
    # Initialize system
    targeting_system = HybridTargetingSystem()
    
    try:
        # Start system
        if not targeting_system.start_system():
            return
        
        # Wait for system to stabilize
        time.sleep(1)
        
        # Create mock detections
        mock_detections = create_mock_detections()
        
        print(f"Mock detections created: {len(mock_detections)} total")
        
        # Process detections
        targeting_system.process_frame_detections(mock_detections)
        
        # Wait for static sequence to complete
        time.sleep(5)
        
        # Test manual targeting
        print("\n=== Manual Targeting Test ===")
        targeting_system.manual_target_test(320, 240)  # Center of image
        targeting_system.manual_target_test(160, 120)  # Upper left quadrant
        targeting_system.manual_target_test(480, 360)  # Lower right quadrant
        
        # Return to home
        targeting_system.motor_system.move_to_target(0.0, 0.0)
        
        print("\n=== Demo Complete ===")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"Demo error: {e}")
    finally:
        targeting_system.stop_system()


if __name__ == "__main__":
    main() 