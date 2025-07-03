#!/usr/bin/env python3
"""
Stepper Motor Controller for Air Defense System
Jetson Nano GPIO control for dual stepper motor setup
"""

import time
import threading
import json
from typing import Tuple, Optional
import Jetson.GPIO as GPIO

class StepperMotorController:
    """
    Stepper motor controller with position tracking and safety limits
    """
    
    def __init__(self, name: str, step_pin: int, dir_pin: int, enable_pin: int, 
                 steps_per_rev: int = 200, max_speed: int = 2000, microstep: int = 1):
        """
        Initialize stepper motor controller
        
        Args:
            name: Motor identifier ("base" or "elevation")
            step_pin: GPIO pin for step pulses
            dir_pin: GPIO pin for direction control
            enable_pin: GPIO pin for motor enable/disable
            steps_per_rev: Steps per full revolution (typically 200 for 1.8° motors)
            max_speed: Maximum steps per second
            microstep: Microstepping factor (1, 2, 4, 8, 16)
        """
        self.name = name
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self.steps_per_rev = steps_per_rev * microstep
        self.max_speed = max_speed
        self.microstep = microstep
        
        # Position tracking
        self.current_position = 0  # Steps from home
        self.current_angle = 0.0   # Degrees from home
        self.is_homed = False
        
        # Safety limits (degrees)
        if name == "base":
            self.min_angle = -180.0
            self.max_angle = 180.0
        else:  # elevation
            self.min_angle = -90.0
            self.max_angle = 90.0
        
        # Threading
        self.is_moving = False
        self.movement_lock = threading.Lock()
        
        # Initialize GPIO
        self._setup_gpio()
        
    def _setup_gpio(self):
        """Initialize GPIO pins"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        
        # Start with motor disabled
        GPIO.output(self.enable_pin, GPIO.HIGH)  # HIGH = disabled for most drivers
        GPIO.output(self.step_pin, GPIO.LOW)
        GPIO.output(self.dir_pin, GPIO.LOW)
        
        print(f"[{self.name}] GPIO pins initialized: STEP={self.step_pin}, DIR={self.dir_pin}, EN={self.enable_pin}")
    
    def enable_motor(self):
        """Enable the stepper motor"""
        GPIO.output(self.enable_pin, GPIO.LOW)  # LOW = enabled
        time.sleep(0.1)  # Give driver time to stabilize
        print(f"[{self.name}] Motor enabled")
    
    def disable_motor(self):
        """Disable the stepper motor"""
        GPIO.output(self.enable_pin, GPIO.HIGH)  # HIGH = disabled
        print(f"[{self.name}] Motor disabled")
    
    def _check_limits(self, target_angle: float) -> bool:
        """Check if target angle is within safety limits"""
        if target_angle < self.min_angle or target_angle > self.max_angle:
            print(f"[{self.name}] ERROR: Target angle {target_angle}° outside limits [{self.min_angle}°, {self.max_angle}°]")
            return False
        return True
    
    def _steps_for_angle(self, angle: float) -> int:
        """Convert angle to steps"""
        return int((angle / 360.0) * self.steps_per_rev)
    
    def _angle_for_steps(self, steps: int) -> float:
        """Convert steps to angle"""
        return (steps / self.steps_per_rev) * 360.0
    
    def move_steps(self, steps: int, speed: int = None) -> bool:
        """
        Move motor by specified number of steps
        
        Args:
            steps: Number of steps (positive = clockwise, negative = counterclockwise)
            speed: Steps per second (default: max_speed/2)
        
        Returns:
            bool: True if movement successful
        """
        if speed is None:
            speed = self.max_speed // 2
        
        if speed > self.max_speed:
            speed = self.max_speed
        
        with self.movement_lock:
            if self.is_moving:
                print(f"[{self.name}] WARNING: Motor already moving!")
                return False
            
            self.is_moving = True
        
        try:
            # Check target position limits
            target_angle = self.current_angle + self._angle_for_steps(steps)
            if not self._check_limits(target_angle):
                return False
            
            # Enable motor
            self.enable_motor()
            
            # Set direction
            if steps >= 0:
                GPIO.output(self.dir_pin, GPIO.HIGH)  # Clockwise
                direction = "CW"
            else:
                GPIO.output(self.dir_pin, GPIO.LOW)   # Counterclockwise
                direction = "CCW"
                steps = abs(steps)
            
            # Calculate timing
            delay = 1.0 / (2.0 * speed)  # Half period in seconds
            
            print(f"[{self.name}] Moving {steps} steps {direction} at {speed} steps/sec")
            
            # Generate step pulses
            for i in range(steps):
                GPIO.output(self.step_pin, GPIO.HIGH)
                time.sleep(delay)
                GPIO.output(self.step_pin, GPIO.LOW)
                time.sleep(delay)
                
                # Update position tracking
                if direction == "CW":
                    self.current_position += 1
                else:
                    self.current_position -= 1
            
            # Update angle tracking
            self.current_angle = self._angle_for_steps(self.current_position)
            
            print(f"[{self.name}] Movement complete. Position: {self.current_angle:.2f}°")
            return True
            
        except Exception as e:
            print(f"[{self.name}] ERROR during movement: {e}")
            return False
        finally:
            self.is_moving = False
    
    def move_to_angle(self, target_angle: float, speed: int = None) -> bool:
        """
        Move motor to absolute angle position
        
        Args:
            target_angle: Target angle in degrees
            speed: Steps per second
        
        Returns:
            bool: True if movement successful
        """
        if not self._check_limits(target_angle):
            return False
        
        angle_diff = target_angle - self.current_angle
        steps_needed = self._steps_for_angle(angle_diff)
        
        return self.move_steps(steps_needed, speed)
    
    def home_motor(self) -> bool:
        """
        Home the motor to 0 position
        Note: This is software homing only. For hardware limit switches,
        this method should be extended.
        """
        print(f"[{self.name}] Homing motor...")
        
        # Simple software homing - go to 0 angle
        success = self.move_to_angle(0.0, speed=self.max_speed // 4)
        
        if success:
            self.current_position = 0
            self.current_angle = 0.0
            self.is_homed = True
            print(f"[{self.name}] Homing complete")
        else:
            print(f"[{self.name}] Homing failed!")
        
        return success
    
    def get_position(self) -> Tuple[float, int]:
        """Get current position"""
        return self.current_angle, self.current_position
    
    def emergency_stop(self):
        """Emergency stop - immediately disable motor"""
        self.disable_motor()
        self.is_moving = False
        print(f"[{self.name}] EMERGENCY STOP!")
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.disable_motor()
        # Note: Don't call GPIO.cleanup() here as other motors might be using GPIO


class DualMotorSystem:
    """
    Coordinated control of base (azimuth) and elevation motors
    """
    
    def __init__(self, config_file: str = None):
        """Initialize dual motor system with configuration"""
        
        # Default pin configuration for Jetson Nano
        default_config = {
            "base_motor": {
                "step_pin": 18,
                "dir_pin": 19,
                "enable_pin": 20,
                "max_speed": 2000
            },
            "elevation_motor": {
                "step_pin": 21,
                "dir_pin": 22,
                "enable_pin": 23,
                "max_speed": 1500
            },
            "homing_speed": 500
        }
        
        if config_file:
            with open(config_file, 'r') as f:
                config = json.load(f)
        else:
            config = default_config
        
        # Initialize motors
        self.base_motor = StepperMotorController(
            name="base",
            step_pin=config["base_motor"]["step_pin"],
            dir_pin=config["base_motor"]["dir_pin"],
            enable_pin=config["base_motor"]["enable_pin"],
            max_speed=config["base_motor"]["max_speed"]
        )
        
        self.elevation_motor = StepperMotorController(
            name="elevation",
            step_pin=config["elevation_motor"]["step_pin"],
            dir_pin=config["elevation_motor"]["dir_pin"],
            enable_pin=config["elevation_motor"]["enable_pin"],
            max_speed=config["elevation_motor"]["max_speed"]
        )
        
        self.homing_speed = config["homing_speed"]
        
    def home_all_motors(self) -> bool:
        """Home both motors sequentially"""
        print("Starting system homing sequence...")
        
        # Home base motor first
        base_success = self.base_motor.home_motor()
        time.sleep(0.5)
        
        # Then elevation motor
        elevation_success = self.elevation_motor.home_motor()
        
        success = base_success and elevation_success
        if success:
            print("System homing complete!")
        else:
            print("System homing failed!")
        
        return success
    
    def move_to_target(self, azimuth: float, elevation: float, speed: int = None) -> bool:
        """
        Move both motors to target position
        
        Args:
            azimuth: Base motor angle (degrees)
            elevation: Elevation motor angle (degrees)
            speed: Movement speed (steps/sec)
        
        Returns:
            bool: True if both movements successful
        """
        print(f"Moving to target: Azimuth={azimuth:.2f}°, Elevation={elevation:.2f}°")
        
        # Move both motors simultaneously using threading
        base_thread = threading.Thread(
            target=lambda: self.base_motor.move_to_angle(azimuth, speed)
        )
        elevation_thread = threading.Thread(
            target=lambda: self.elevation_motor.move_to_angle(elevation, speed)
        )
        
        base_thread.start()
        elevation_thread.start()
        
        base_thread.join()
        elevation_thread.join()
        
        # Check if both movements were successful
        base_angle, _ = self.base_motor.get_position()
        elev_angle, _ = self.elevation_motor.get_position()
        
        success = (abs(base_angle - azimuth) < 1.0 and 
                  abs(elev_angle - elevation) < 1.0)
        
        if success:
            print(f"Target reached successfully!")
        else:
            print(f"Target positioning failed!")
        
        return success
    
    def get_current_position(self) -> Tuple[float, float]:
        """Get current position of both motors"""
        base_angle, _ = self.base_motor.get_position()
        elev_angle, _ = self.elevation_motor.get_position()
        return base_angle, elev_angle
    
    def emergency_stop_all(self):
        """Emergency stop for all motors"""
        print("EMERGENCY STOP - ALL MOTORS")
        self.base_motor.emergency_stop()
        self.elevation_motor.emergency_stop()
    
    def cleanup(self):
        """Clean up all motor resources"""
        self.base_motor.cleanup()
        self.elevation_motor.cleanup()
        GPIO.cleanup()
        print("Motor system cleanup complete")


if __name__ == "__main__":
    # Test script
    try:
        print("Initializing dual motor system...")
        motor_system = DualMotorSystem()
        
        print("Homing motors...")
        motor_system.home_all_motors()
        
        print("Testing movement...")
        motor_system.move_to_target(45.0, 30.0)
        time.sleep(2)
        
        motor_system.move_to_target(-45.0, -15.0)
        time.sleep(2)
        
        print("Returning to home...")
        motor_system.move_to_target(0.0, 0.0)
        
    except KeyboardInterrupt:
        print("\nUser interrupted!")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        motor_system.cleanup() 