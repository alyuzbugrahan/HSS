#!/usr/bin/env python3
"""
Firing Controller Node for ROS2 Air Defense System
Controls weapon firing system with comprehensive safety measures
"""

import sys
import time
import threading
from pathlib import Path
from typing import Optional

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Custom message imports
from ros2_air_defense.msg import FiringCommand, SafetyStatus, SystemStatus
from ros2_air_defense.srv import ArmSystem, DisarmSystem, EmergencyStop

# Import existing firing system
project_root = Path(__file__).parent.parent.parent
sys.path.append(str(project_root))

try:
    from src.shooting.firing_controller import PneumaticFiringController
    FIRING_SYSTEM_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Firing system not available: {e}")
    FIRING_SYSTEM_AVAILABLE = False

class FiringControllerNode(Node):
    """
    ROS2 node for weapon firing control with comprehensive safety systems
    """
    
    def __init__(self):
        super().__init__('firing_controller_node')
        
        # Node parameters
        self.declare_parameter('firing_config_path', 'config/firing_config.json')
        self.declare_parameter('safety_update_rate', 10.0)  # Hz
        self.declare_parameter('enable_gpio', True)
        self.declare_parameter('require_authorization', True)
        self.declare_parameter('max_consecutive_shots', 5)
        self.declare_parameter('min_fire_interval', 1.0)  # seconds
        
        # Get parameters
        firing_config_path = self.get_parameter('firing_config_path').value
        self.safety_rate = self.get_parameter('safety_update_rate').value
        self.enable_gpio = self.get_parameter('enable_gpio').value
        self.require_auth = self.get_parameter('require_authorization').value
        self.max_consecutive = self.get_parameter('max_consecutive_shots').value
        self.min_fire_interval = self.get_parameter('min_fire_interval').value
        
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.safety_status_pub = self.create_publisher(
            SafetyStatus,
            'safety_status',
            reliable_qos
        )
        
        self.system_status_pub = self.create_publisher(
            SystemStatus,
            'firing_system_status',
            reliable_qos
        )
        
        # Subscribers
        self.firing_command_sub = self.create_subscription(
            FiringCommand,
            'firing_commands',
            self.firing_command_callback,
            reliable_qos
        )
        
        # Service servers
        self.arm_service = self.create_service(
            ArmSystem,
            'arm_firing_system',
            self.arm_system_callback
        )
        
        self.disarm_service = self.create_service(
            DisarmSystem,
            'disarm_firing_system',
            self.disarm_system_callback
        )
        
        self.emergency_stop_service = self.create_service(
            EmergencyStop,
            'emergency_stop_firing',
            self.emergency_stop_callback
        )
        
        # Initialize firing system
        self.firing_system = None
        self.system_ready = False
        
        if FIRING_SYSTEM_AVAILABLE and self.enable_gpio:
            self._initialize_firing_system(firing_config_path)
        else:
            self.get_logger().warn("Firing system running in simulation mode")
            self._initialize_simulation_mode()
        
        # Safety and operational state
        self.armed = False
        self.emergency_stop_active = False
        self.safety_override_active = False
        self.session_id = 0
        self.armed_at = None
        
        # Firing statistics
        self.shots_fired_total = 0
        self.shots_fired_session = 0
        self.consecutive_shots = 0
        self.last_fire_time = 0.0
        self.successful_engagements = 0
        self.failed_engagements = 0
        
        # Safety state
        self.safety_status = SafetyStatus()
        self.safety_violations = []
        self.safety_warnings = []
        
        # Threading
        self.firing_lock = threading.Lock()
        
        # Timers
        safety_timer_period = 1.0 / self.safety_rate
        self.safety_timer = self.create_timer(safety_timer_period, self.update_safety_status)
        self.status_timer = self.create_timer(1.0, self.publish_system_status)
        
        self.get_logger().info(f"Firing Controller Node initialized")
        self.get_logger().info(f"  GPIO enabled: {self.enable_gpio}")
        self.get_logger().info(f"  Authorization required: {self.require_auth}")
        self.get_logger().info(f"  Max consecutive shots: {self.max_consecutive}")
    
    def _initialize_firing_system(self, config_path: str):
        """Initialize the pneumatic firing system"""
        try:
            self.firing_system = PneumaticFiringController(config_path)
            self.system_ready = True
            self.get_logger().info(f"✅ Firing system initialized with config: {config_path}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Firing system initialization failed: {e}")
            self._initialize_simulation_mode()
    
    def _initialize_simulation_mode(self):
        """Initialize simulation mode"""
        self.firing_system = None
        self.system_ready = True
        self.get_logger().info("✅ Firing system initialized in simulation mode")
    
    def firing_command_callback(self, msg: FiringCommand):
        """Handle incoming firing commands"""
        if self.emergency_stop_active:
            self.get_logger().warn("Emergency stop active - ignoring firing command")
            return
        
        if not self.armed and msg.command_type != FiringCommand.ARM_WEAPON:
            self.get_logger().warn("System not armed - ignoring firing command")
            return
        
        self.get_logger().info(f"Received firing command: {msg.command_type}")
        
        if msg.command_type == FiringCommand.FIRE_SINGLE:
            self._handle_fire_single(msg)
        elif msg.command_type == FiringCommand.FIRE_BURST:
            self._handle_fire_burst(msg)
        elif msg.command_type == FiringCommand.ARM_WEAPON:
            self._handle_arm_weapon(msg)
        elif msg.command_type == FiringCommand.DISARM_WEAPON:
            self._handle_disarm_weapon(msg)
        elif msg.command_type == FiringCommand.TEST_FIRING:
            self._handle_test_firing(msg)
        elif msg.command_type == FiringCommand.EMERGENCY_CEASE:
            self._handle_emergency_cease(msg)
        else:
            self.get_logger().warn(f"Unknown firing command: {msg.command_type}")
    
    def _handle_fire_single(self, msg: FiringCommand):
        """Handle single shot firing"""
        if not self._check_safety_for_firing(msg):
            return
        
        with self.firing_lock:
            current_time = time.time()
            
            # Check firing rate limit
            if current_time - self.last_fire_time < self.min_fire_interval:
                self.get_logger().warn("Firing rate limit exceeded")
                return
            
            # Check consecutive shots limit
            if self.consecutive_shots >= self.max_consecutive:
                self.get_logger().warn("Maximum consecutive shots reached")
                return
            
            # Execute firing
            if self.firing_system:
                try:
                    success = self.firing_system.fire_single(
                        duration=msg.valve_duration,
                        pressure=msg.pressure_setting
                    )
                except Exception as e:
                    self.get_logger().error(f"Firing error: {e}")
                    success = False
            else:
                # Simulation
                self.get_logger().info(f"[SIM] Firing single shot at target {msg.target_id}")
                success = True
                time.sleep(0.1)  # Simulate firing delay
            
            # Update statistics
            if success:
                self.shots_fired_total += 1
                self.shots_fired_session += 1
                self.consecutive_shots += 1
                self.last_fire_time = current_time
                
                self.get_logger().info(
                    f"✅ Shot fired successfully at target {msg.target_id} "
                    f"(Az={msg.target_azimuth:.1f}°, El={msg.target_elevation:.1f}°)"
                )
                
                # Reset consecutive counter after brief pause
                self.create_timer(
                    self.min_fire_interval * 2, 
                    lambda: setattr(self, 'consecutive_shots', 0),
                    clock=self.get_clock()
                )
                
            else:
                self.failed_engagements += 1
                self.get_logger().error("❌ Firing failed")
    
    def _handle_fire_burst(self, msg: FiringCommand):
        """Handle burst firing"""
        if not self._check_safety_for_firing(msg):
            return
        
        shot_count = min(msg.shot_count, self.max_consecutive)
        self.get_logger().info(f"Executing burst fire: {shot_count} shots")
        
        # Execute burst in separate thread to avoid blocking
        def burst_thread():
            for i in range(shot_count):
                if self.emergency_stop_active or not self.armed:
                    break
                
                # Create individual shot command
                single_shot = FiringCommand()
                single_shot.command_type = FiringCommand.FIRE_SINGLE
                single_shot.target_id = msg.target_id
                single_shot.target_azimuth = msg.target_azimuth
                single_shot.target_elevation = msg.target_elevation
                single_shot.valve_duration = msg.valve_duration
                single_shot.pressure_setting = msg.pressure_setting
                
                self._handle_fire_single(single_shot)
                
                if i < shot_count - 1:  # Don't delay after last shot
                    time.sleep(msg.firing_delay)
        
        thread = threading.Thread(target=burst_thread)
        thread.daemon = True
        thread.start()
    
    def _handle_arm_weapon(self, msg: FiringCommand):
        """Handle weapon arming"""
        if self.require_auth and not msg.manual_authorization:
            self.get_logger().warn("Manual authorization required for arming")
            return
        
        if not self._check_overall_safety():
            self.get_logger().error("Safety check failed - cannot arm weapon")
            return
        
        self.armed = True
        self.session_id += 1
        self.armed_at = self.get_clock().now()
        self.shots_fired_session = 0
        
        self.get_logger().info(f"✅ Weapon system ARMED (Session {self.session_id})")
    
    def _handle_disarm_weapon(self, msg: FiringCommand):
        """Handle weapon disarming"""
        self.armed = False
        session_duration = None
        
        if self.armed_at:
            session_duration = self.get_clock().now() - self.armed_at
        
        self.get_logger().info(
            f"✅ Weapon system DISARMED "
            f"(Session {self.session_id}, Shots: {self.shots_fired_session})"
        )
    
    def _handle_test_firing(self, msg: FiringCommand):
        """Handle test firing (no projectile)"""
        if self.firing_system:
            try:
                success = self.firing_system.test_fire()
                self.get_logger().info("✅ Test firing completed" if success else "❌ Test firing failed")
            except Exception as e:
                self.get_logger().error(f"Test firing error: {e}")
        else:
            self.get_logger().info("[SIM] Test firing completed")
    
    def _handle_emergency_cease(self, msg: FiringCommand):
        """Handle emergency cease fire"""
        self.get_logger().error("🚨 EMERGENCY CEASE FIRE")
        
        self.armed = False
        self.emergency_stop_active = True
        
        if self.firing_system:
            try:
                self.firing_system.emergency_stop()
            except Exception as e:
                self.get_logger().error(f"Emergency cease error: {e}")
    
    def _check_safety_for_firing(self, msg: FiringCommand) -> bool:
        """Comprehensive safety check before firing"""
        if not self.armed:
            self.get_logger().warn("System not armed")
            return False
        
        if self.emergency_stop_active:
            self.get_logger().warn("Emergency stop active")
            return False
        
        if not self._check_overall_safety():
            self.get_logger().warn("Safety check failed")
            return False
        
        # Check target authorization
        if self.require_auth and not msg.manual_authorization:
            self.get_logger().warn("Target engagement not authorized")
            return False
        
        # Check friendly fire
        if not msg.confirm_target:
            self.get_logger().warn("Target not confirmed")
            return False
        
        return True
    
    def _check_overall_safety(self) -> bool:
        """Check overall system safety"""
        # This would integrate with actual safety sensors
        # For now, return True if no emergency stop
        return not self.emergency_stop_active
    
    def arm_system_callback(self, request, response):
        """Handle arm system service"""
        self.get_logger().info(f"Arm system request from {request.operator_id}")
        
        if not self._check_overall_safety():
            response.success = False
            response.message = "Safety check failed"
            return response
        
        if request.force_arm or not self.require_auth:
            self.armed = True
            self.session_id += 1
            self.armed_at = self.get_clock().now()
            self.shots_fired_session = 0
            
            response.success = True
            response.already_armed = False
            response.session_id = self.session_id
            response.armed_at = self.armed_at.to_msg()
            response.message = "System armed successfully"
            
            # Update safety status
            self.safety_status.safety_switch_engaged = True
            
        else:
            response.success = False
            response.message = "Authorization required"
        
        return response
    
    def disarm_system_callback(self, request, response):
        """Handle disarm system service"""
        self.get_logger().info(f"Disarm system request from {request.operator_id}")
        
        was_armed = self.armed
        self.armed = False
        
        session_duration = None
        if self.armed_at:
            session_duration = self.get_clock().now() - self.armed_at
        
        response.success = True
        response.was_armed = was_armed
        response.disarmed_at = self.get_clock().now().to_msg()
        response.shots_fired_session = self.shots_fired_session
        response.message = "System disarmed successfully"
        
        if session_duration:
            response.session_duration = session_duration.to_msg()
        
        return response
    
    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service"""
        self.get_logger().error(f"🚨 Emergency stop firing: {request.reason}")
        
        self.emergency_stop_active = True
        self.armed = False
        
        if self.firing_system:
            try:
                self.firing_system.emergency_stop()
                response.success = True
                response.systems_stopped = 1
            except Exception as e:
                response.success = False
                response.systems_stopped = 0
        else:
            response.success = True
            response.systems_stopped = 1
        
        response.message = "Emergency stop executed"
        response.stopped_at = self.get_clock().now().to_msg()
        
        return response
    
    def update_safety_status(self):
        """Update and publish safety status"""
        current_time = self.get_clock().now()
        
        # Update safety status message
        self.safety_status.header.stamp = current_time.to_msg()
        
        # Determine safety level
        if self.emergency_stop_active:
            self.safety_status.safety_level = SafetyStatus.SAFETY_EMERGENCY
        elif len(self.safety_violations) > 0:
            self.safety_status.safety_level = SafetyStatus.SAFETY_CRITICAL
        elif len(self.safety_warnings) > 0:
            self.safety_status.safety_level = SafetyStatus.SAFETY_WARNING
        else:
            self.safety_status.safety_level = SafetyStatus.SAFETY_SECURE
        
        # Update safety systems status
        self.safety_status.safety_switch_engaged = self.armed
        self.safety_status.emergency_stop_clear = not self.emergency_stop_active
        
        # Update timing safety
        current_time_sec = current_time.nanoseconds / 1e9
        time_since_fire = current_time_sec - self.last_fire_time
        self.safety_status.time_since_last_fire.sec = int(time_since_fire)
        
        # Update firing rate safety
        self.safety_status.firing_rate_ok = (time_since_fire >= self.min_fire_interval)
        self.safety_status.consecutive_shots = self.consecutive_shots
        
        # Update violations and warnings
        self.safety_status.active_violations = len(self.safety_violations)
        self.safety_status.warning_count = len(self.safety_warnings)
        self.safety_status.violations = self.safety_violations
        self.safety_status.warnings = self.safety_warnings
        
        # Update interlocks
        self.safety_status.weapon_safety_ok = self.system_ready and not self.emergency_stop_active
        self.safety_status.all_interlocks_ok = self.safety_status.weapon_safety_ok
        
        # Publish safety status
        self.safety_status_pub.publish(self.safety_status)
    
    def publish_system_status(self):
        """Publish system status"""
        msg = SystemStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.system_name = "Firing Controller"
        msg.version = "1.0.0"
        
        # Determine system state
        if self.emergency_stop_active:
            msg.system_state = SystemStatus.STATE_EMERGENCY
        elif self.armed:
            msg.system_state = SystemStatus.STATE_ARMED
        elif self.system_ready:
            msg.system_state = SystemStatus.STATE_STANDBY
        else:
            msg.system_state = SystemStatus.STATE_ERROR
        
        # Update subsystem status
        msg.firing_system_online = self.system_ready
        
        # Update statistics
        msg.total_shots_fired = self.shots_fired_total
        msg.successful_engagements = self.successful_engagements
        msg.failed_engagements = self.failed_engagements
        
        # Current mode
        if self.armed:
            msg.current_mode = "armed"
        else:
            msg.current_mode = "disarmed"
        
        self.system_status_pub.publish(msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = FiringControllerNode()
        
        print("🔫 Firing Controller Node started!")
        print("   Subscribing to: /firing_commands")
        print("   Publishing safety status to: /safety_status")
        print("   Services: /arm_firing_system, /disarm_firing_system, /emergency_stop_firing")
        print("   ⚠️  WEAPON SYSTEM ACTIVE - EXTREME CAUTION ⚠️")
        print("   Press Ctrl+C to stop...")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Firing Controller Node stopped by user")
    except Exception as e:
        print(f"❌ Firing Controller Node error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 