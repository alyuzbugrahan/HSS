#!/usr/bin/env python3
"""
Firing Controller for Air Defense System
Controls pneumatic valve system for paintball-style projectile firing
"""

import time
import threading
from typing import Optional, Callable
import json
from enum import Enum
from dataclasses import dataclass

try:
    import Jetson.GPIO as GPIO
except ImportError:
    print("⚠️ Jetson.GPIO not available, using mock GPIO")
    # Mock GPIO for testing on non-Jetson platforms
    class MockGPIO:
        BCM = "BCM"
        OUT = "OUT"
        IN = "IN"
        HIGH = 1
        LOW = 0
        
        @staticmethod
        def setmode(mode): pass
        @staticmethod
        def setup(pin, mode, pull_up_down=None): pass
        @staticmethod
        def output(pin, state): pass
        @staticmethod
        def input(pin): return 0
        @staticmethod
        def cleanup(): pass
    
    GPIO = MockGPIO()

class FiringMode(Enum):
    SAFE = "safe"
    SINGLE = "single"
    BURST = "burst"

@dataclass
class FiringConfig:
    """Configuration for firing system"""
    valve_pin: int = 16              # GPIO pin for main firing valve
    pressure_sensor_pin: int = 17    # GPIO pin for pressure sensor (optional)
    safety_pin: int = 18            # GPIO pin for safety switch
    fire_duration: float = 0.1      # Duration to keep valve open (seconds)
    min_fire_interval: float = 1.0  # Minimum time between shots (seconds)
    burst_count: int = 3            # Number of shots in burst mode
    burst_interval: float = 0.2     # Time between shots in burst
    max_pressure: int = 100         # Maximum safe pressure (PSI or arbitrary units)
    safety_timeout: float = 5.0     # Safety timeout after firing (seconds)

class FiringController:
    """
    Controller for pneumatic firing system
    Manages valve control, safety systems, and firing sequences
    """
    
    def __init__(self, config: FiringConfig = None):
        """Initialize firing controller"""
        self.config = config or FiringConfig()
        
        # System state
        self.is_armed = False
        self.is_firing = False
        self.last_fire_time = 0.0
        self.total_shots_fired = 0
        self.current_mode = FiringMode.SAFE
        
        # Safety systems
        self.safety_enabled = True
        self.pressure_ok = True
        self.system_ready = False
        
        # Threading
        self.firing_lock = threading.Lock()
        self.safety_timer = None
        
        # Callbacks
        self.on_fire_complete: Optional[Callable] = None
        self.on_safety_violation: Optional[Callable] = None
        self.on_pressure_warning: Optional[Callable] = None
        
        # Initialize GPIO
        self._setup_gpio()
        
        print(f"🔫 Firing controller initialized")
        print(f"   Valve pin: {self.config.valve_pin}")
        print(f"   Fire duration: {self.config.fire_duration}s")
        print(f"   Min interval: {self.config.min_fire_interval}s")
    
    def _setup_gpio(self):
        """Initialize GPIO pins for firing system"""
        try:
            GPIO.setmode(GPIO.BCM)
            
            # Setup valve control pin (output)
            GPIO.setup(self.config.valve_pin, GPIO.OUT)
            GPIO.output(self.config.valve_pin, GPIO.LOW)  # Valve closed by default
            
            # Setup safety switch pin (input with pull-up)
            GPIO.setup(self.config.safety_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Setup pressure sensor pin (input) - optional
            try:
                GPIO.setup(self.config.pressure_sensor_pin, GPIO.IN)
            except:
                print("⚠️ Pressure sensor pin not configured")
            
            print(f"✅ GPIO pins configured for firing system")
            
        except Exception as e:
            print(f"❌ GPIO setup error: {e}")
            self.system_ready = False
    
    def arm_system(self) -> bool:
        """
        Arm the firing system
        Performs safety checks before allowing firing
        """
        if not self._safety_check():
            print("❌ Safety check failed - cannot arm system")
            return False
        
        self.is_armed = True
        self.current_mode = FiringMode.SINGLE  # Default to single fire
        self.system_ready = True
        
        print("🔫 Firing system ARMED")
        return True
    
    def disarm_system(self):
        """Disarm the firing system"""
        self.is_armed = False
        self.current_mode = FiringMode.SAFE
        self.is_firing = False
        
        # Ensure valve is closed
        GPIO.output(self.config.valve_pin, GPIO.LOW)
        
        print("🔒 Firing system DISARMED")
    
    def set_firing_mode(self, mode: FiringMode) -> bool:
        """Set firing mode"""
        if not self.is_armed and mode != FiringMode.SAFE:
            print("❌ Cannot change firing mode - system not armed")
            return False
        
        self.current_mode = mode
        print(f"🎯 Firing mode set to: {mode.value}")
        return True
    
    def _safety_check(self) -> bool:
        """Perform comprehensive safety check"""
        safety_issues = []
        
        # Check safety switch
        try:
            safety_switch_state = GPIO.input(self.config.safety_pin)
            if safety_switch_state == GPIO.LOW:  # Assuming LOW = safety off
                safety_issues.append("Safety switch not engaged")
        except:
            safety_issues.append("Cannot read safety switch")
        
        # Check pressure (if sensor available)
        try:
            pressure_reading = self._read_pressure()
            if pressure_reading > self.config.max_pressure:
                safety_issues.append(f"Pressure too high: {pressure_reading}")
            elif pressure_reading < 10:  # Minimum operational pressure
                safety_issues.append(f"Pressure too low: {pressure_reading}")
        except:
            print("⚠️ Pressure sensor not available")
        
        # Check timing constraints
        time_since_last_fire = time.time() - self.last_fire_time
        if time_since_last_fire < self.config.min_fire_interval:
            safety_issues.append(f"Too soon since last fire: {time_since_last_fire:.1f}s")
        
        # Report safety issues
        if safety_issues:
            print("🚨 Safety violations:")
            for issue in safety_issues:
                print(f"   - {issue}")
            return False
        
        return True
    
    def _read_pressure(self) -> int:
        """Read pressure sensor (mock implementation)"""
        # In real implementation, this would read from ADC or pressure sensor
        # For now, return a safe value
        return 50  # Mock pressure reading
    
    def fire_single_shot(self) -> bool:
        """
        Fire a single shot
        
        Returns:
            bool: True if shot was fired successfully
        """
        if not self._can_fire():
            return False
        
        with self.firing_lock:
            if self.is_firing:
                print("❌ Already firing")
                return False
            
            self.is_firing = True
        
        try:
            print("🔥 FIRING SINGLE SHOT")
            
            # Open valve
            GPIO.output(self.config.valve_pin, GPIO.HIGH)
            
            # Keep valve open for specified duration
            time.sleep(self.config.fire_duration)
            
            # Close valve
            GPIO.output(self.config.valve_pin, GPIO.LOW)
            
            # Update counters
            self.total_shots_fired += 1
            self.last_fire_time = time.time()
            
            print(f"✅ Shot fired successfully (Total: {self.total_shots_fired})")
            
            # Trigger callback if set
            if self.on_fire_complete:
                self.on_fire_complete(1, True, "Single shot fired")
            
            # Start safety timer
            self._start_safety_timer()
            
            return True
            
        except Exception as e:
            print(f"❌ Firing error: {e}")
            return False
            
        finally:
            self.is_firing = False
            # Ensure valve is closed
            GPIO.output(self.config.valve_pin, GPIO.LOW)
    
    def fire_burst(self) -> bool:
        """
        Fire a burst of shots
        
        Returns:
            bool: True if burst was completed successfully
        """
        if not self._can_fire():
            return False
        
        if self.current_mode != FiringMode.BURST:
            print("❌ Not in burst mode")
            return False
        
        with self.firing_lock:
            if self.is_firing:
                print("❌ Already firing")
                return False
            
            self.is_firing = True
        
        try:
            print(f"🔥 FIRING BURST ({self.config.burst_count} shots)")
            
            shots_fired = 0
            for i in range(self.config.burst_count):
                if not self._can_fire(skip_interval_check=True):
                    break
                
                # Open valve
                GPIO.output(self.config.valve_pin, GPIO.HIGH)
                time.sleep(self.config.fire_duration)
                GPIO.output(self.config.valve_pin, GPIO.LOW)
                
                shots_fired += 1
                self.total_shots_fired += 1
                
                print(f"   Shot {i+1}/{self.config.burst_count}")
                
                # Wait between shots (except for last shot)
                if i < self.config.burst_count - 1:
                    time.sleep(self.config.burst_interval)
            
            self.last_fire_time = time.time()
            
            print(f"✅ Burst complete: {shots_fired}/{self.config.burst_count} shots")
            
            # Trigger callback
            if self.on_fire_complete:
                self.on_fire_complete(shots_fired, shots_fired == self.config.burst_count, 
                                    f"Burst fired: {shots_fired} shots")
            
            # Start safety timer
            self._start_safety_timer()
            
            return shots_fired == self.config.burst_count
            
        except Exception as e:
            print(f"❌ Burst firing error: {e}")
            return False
            
        finally:
            self.is_firing = False
            GPIO.output(self.config.valve_pin, GPIO.LOW)
    
    def _can_fire(self, skip_interval_check: bool = False) -> bool:
        """Check if system can fire"""
        if not self.is_armed:
            print("❌ System not armed")
            return False
        
        if self.current_mode == FiringMode.SAFE:
            print("❌ System in safe mode")
            return False
        
        if not skip_interval_check:
            time_since_last = time.time() - self.last_fire_time
            if time_since_last < self.config.min_fire_interval:
                print(f"❌ Too soon since last fire: {time_since_last:.1f}s")
                return False
        
        if not self._safety_check():
            return False
        
        return True
    
    def _start_safety_timer(self):
        """Start safety timer after firing"""
        if self.safety_timer:
            self.safety_timer.cancel()
        
        def safety_timeout():
            print("🔒 Safety timeout - system auto-disarmed")
            self.disarm_system()
        
        self.safety_timer = threading.Timer(self.config.safety_timeout, safety_timeout)
        self.safety_timer.start()
    
    def emergency_stop(self):
        """Emergency stop - immediately stop all firing operations"""
        print("🚨 EMERGENCY STOP - FIRING SYSTEM")
        
        # Immediately close valve
        GPIO.output(self.config.valve_pin, GPIO.LOW)
        
        # Disarm system
        self.disarm_system()
        
        # Cancel safety timer
        if self.safety_timer:
            self.safety_timer.cancel()
        
        # Trigger safety violation callback
        if self.on_safety_violation:
            self.on_safety_violation("Emergency stop activated")
    
    def get_system_status(self) -> dict:
        """Get comprehensive system status"""
        return {
            "armed": self.is_armed,
            "firing": self.is_firing,
            "mode": self.current_mode.value,
            "ready": self.system_ready,
            "shots_fired": self.total_shots_fired,
            "last_fire_time": self.last_fire_time,
            "time_since_last_fire": time.time() - self.last_fire_time,
            "pressure_ok": self.pressure_ok,
            "safety_enabled": self.safety_enabled
        }
    
    def test_valve(self) -> bool:
        """Test valve operation (safe test without projectile)"""
        if self.is_armed:
            print("❌ Cannot test valve while armed")
            return False
        
        print("🧪 Testing valve operation...")
        
        try:
            # Brief valve operation
            GPIO.output(self.config.valve_pin, GPIO.HIGH)
            time.sleep(0.05)  # Very brief test
            GPIO.output(self.config.valve_pin, GPIO.LOW)
            
            print("✅ Valve test successful")
            return True
            
        except Exception as e:
            print(f"❌ Valve test failed: {e}")
            return False
    
    def cleanup(self):
        """Clean up resources"""
        print("🔄 Cleaning up firing controller...")
        
        # Disarm system
        self.disarm_system()
        
        # Cancel timers
        if self.safety_timer:
            self.safety_timer.cancel()
        
        # Cleanup GPIO
        try:
            GPIO.output(self.config.valve_pin, GPIO.LOW)
        except:
            pass
        
        print("✅ Firing controller cleanup complete")


class WeaponSystem:
    """
    High-level weapon system that integrates firing controller with targeting
    """
    
    def __init__(self, firing_config: FiringConfig = None):
        """Initialize weapon system"""
        self.firing_controller = FiringController(firing_config)
        
        # Weapon state
        self.engagement_mode = "manual"  # manual, auto, semi-auto
        self.target_engaged = False
        self.last_engagement_time = 0.0
        
        print("🎯 Weapon system initialized")
    
    def engage_target(self, target_priority: int = 1) -> bool:
        """
        Engage a target (called after successful targeting)
        
        Args:
            target_priority: Priority level of target (1=highest)
        
        Returns:
            bool: True if engagement was successful
        """
        if not self.firing_controller.is_armed:
            print("❌ Weapon not armed - cannot engage")
            return False
        
        if self.target_engaged:
            print("❌ Already engaging target")
            return False
        
        print(f"🎯 Engaging target (Priority: {target_priority})")
        
        self.target_engaged = True
        
        try:
            # Fire based on current mode
            if self.firing_controller.current_mode == FiringMode.SINGLE:
                success = self.firing_controller.fire_single_shot()
            elif self.firing_controller.current_mode == FiringMode.BURST:
                success = self.firing_controller.fire_burst()
            else:
                print("❌ Invalid firing mode for engagement")
                success = False
            
            if success:
                self.last_engagement_time = time.time()
                print("✅ Target engagement complete")
            else:
                print("❌ Target engagement failed")
            
            return success
            
        finally:
            self.target_engaged = False
    
    def set_engagement_mode(self, mode: str) -> bool:
        """Set engagement mode"""
        valid_modes = ["manual", "auto", "semi-auto"]
        if mode not in valid_modes:
            print(f"❌ Invalid engagement mode: {mode}")
            return False
        
        self.engagement_mode = mode
        print(f"🎯 Engagement mode set to: {mode}")
        return True
    
    def arm_weapon_system(self) -> bool:
        """Arm the complete weapon system"""
        success = self.firing_controller.arm_system()
        if success:
            print("🔫 WEAPON SYSTEM ARMED")
        return success
    
    def disarm_weapon_system(self):
        """Disarm the complete weapon system"""
        self.firing_controller.disarm_system()
        self.target_engaged = False
        print("🔒 WEAPON SYSTEM DISARMED")
    
    def emergency_stop_all(self):
        """Emergency stop entire weapon system"""
        self.firing_controller.emergency_stop()
        self.target_engaged = False
        print("🚨 WEAPON SYSTEM EMERGENCY STOP")
    
    def get_weapon_status(self) -> dict:
        """Get complete weapon system status"""
        firing_status = self.firing_controller.get_system_status()
        
        weapon_status = {
            "engagement_mode": self.engagement_mode,
            "target_engaged": self.target_engaged,
            "last_engagement_time": self.last_engagement_time,
            "ready_to_engage": (firing_status["armed"] and 
                              firing_status["ready"] and 
                              not self.target_engaged)
        }
        
        return {**firing_status, **weapon_status}
    
    def cleanup(self):
        """Clean up weapon system"""
        self.disarm_weapon_system()
        self.firing_controller.cleanup()


# Factory function for easy configuration
def create_weapon_system(valve_pin: int = 16, fire_duration: float = 0.1) -> WeaponSystem:
    """Create a weapon system with specified configuration"""
    config = FiringConfig(
        valve_pin=valve_pin,
        fire_duration=fire_duration,
        min_fire_interval=1.0
    )
    
    return WeaponSystem(config)


if __name__ == "__main__":
    # Test script
    print("🧪 Testing Firing Controller...")
    
    # Create weapon system
    weapon = create_weapon_system()
    
    try:
        # Test valve
        weapon.firing_controller.test_valve()
        
        # Arm system
        weapon.arm_weapon_system()
        
        # Get status
        status = weapon.get_weapon_status()
        print(f"Weapon status: {status}")
        
        # Simulate target engagement
        print("\n🎯 Simulating target engagement...")
        success = weapon.engage_target(target_priority=1)
        
        if success:
            print("✅ Target engagement successful")
        else:
            print("❌ Target engagement failed")
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
    except Exception as e:
        print(f"Test error: {e}")
    finally:
        weapon.cleanup()
    
    print("✅ Firing controller test complete!") 