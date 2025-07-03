#!/usr/bin/env python3
"""
Motor Controller Node for ROS2 Air Defense System
Controls dual stepper motors (azimuth and elevation)
"""

import sys
import time
import threading
from pathlib import Path
from typing import Optional

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Custom message imports
from ros2_air_defense.msg import MotorPosition, MotorCommand, SystemStatus
from ros2_air_defense.action import MoveToPosition
from ros2_air_defense.srv import HomeMotors, EmergencyStop

# Import existing motor system
project_root = Path(__file__).parent.parent.parent
sys.path.append(str(project_root))

try:
    from src.motors.stepper_controller import DualMotorSystem
    MOTOR_SYSTEM_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Motor system not available: {e}")
    MOTOR_SYSTEM_AVAILABLE = False

class MotorControllerNode(Node):
    """
    ROS2 node for controlling dual stepper motor system
    Handles motor movements, homing, and position feedback
    """
    
    def __init__(self):
        super().__init__('motor_controller_node')
        
        # Node parameters
        self.declare_parameter('motor_config_path', 'config/motor_config.json')
        self.declare_parameter('position_update_rate', 50.0)  # Hz
        self.declare_parameter('enable_gpio', True)
        self.declare_parameter('max_azimuth', 180.0)
        self.declare_parameter('max_elevation', 90.0)
        
        # Get parameters
        motor_config_path = self.get_parameter('motor_config_path').value
        self.position_rate = self.get_parameter('position_update_rate').value
        self.enable_gpio = self.get_parameter('enable_gpio').value
        self.max_azimuth = self.get_parameter('max_azimuth').value
        self.max_elevation = self.get_parameter('max_elevation').value
        
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        fast_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.position_pub = self.create_publisher(
            MotorPosition,
            'motor_position',
            fast_qos
        )
        
        self.status_pub = self.create_publisher(
            SystemStatus,
            'motor_system_status',
            reliable_qos
        )
        
        # Subscribers
        self.command_sub = self.create_subscription(
            MotorCommand,
            'motor_commands',
            self.motor_command_callback,
            reliable_qos
        )
        
        # Action servers
        self.move_action_server = ActionServer(
            self,
            MoveToPosition,
            'move_to_position',
            self.move_to_position_callback
        )
        
        # Service servers
        self.home_service = self.create_service(
            HomeMotors,
            'home_motors',
            self.home_motors_callback
        )
        
        self.emergency_stop_service = self.create_service(
            EmergencyStop,
            'emergency_stop',
            self.emergency_stop_callback
        )
        
        # Initialize motor system
        self.motor_system = None
        self.system_ready = False
        self.emergency_stop_active = False
        
        if MOTOR_SYSTEM_AVAILABLE and self.enable_gpio:
            self._initialize_motor_system(motor_config_path)
        else:
            self.get_logger().warn("Motor system running in simulation mode")
            self._initialize_simulation_mode()
        
        # Current state
        self.current_position = MotorPosition()
        self.target_position = {'azimuth': 0.0, 'elevation': 0.0}
        self.is_moving = False
        self.movement_lock = threading.Lock()
        
        # Timers
        position_timer_period = 1.0 / self.position_rate
        self.position_timer = self.create_timer(
            position_timer_period, 
            self.publish_position
        )
        
        # Status timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(f"Motor Controller Node initialized")
        self.get_logger().info(f"  GPIO enabled: {self.enable_gpio}")
        self.get_logger().info(f"  System ready: {self.system_ready}")
        self.get_logger().info(f"  Position rate: {self.position_rate} Hz")
    
    def _initialize_motor_system(self, config_path: str):
        """Initialize the dual motor system"""
        try:
            self.motor_system = DualMotorSystem(config_path)
            self.system_ready = True
            self.get_logger().info(f"✅ Motor system initialized with config: {config_path}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Motor system initialization failed: {e}")
            self._initialize_simulation_mode()
    
    def _initialize_simulation_mode(self):
        """Initialize simulation mode (no actual GPIO)"""
        self.motor_system = None
        self.system_ready = True  # Simulation is always "ready"
        
        # Simulation state
        self.sim_azimuth = 0.0
        self.sim_elevation = 0.0
        self.sim_homed = False
        
        self.get_logger().info("✅ Motor system initialized in simulation mode")
    
    def motor_command_callback(self, msg: MotorCommand):
        """Handle incoming motor commands"""
        if self.emergency_stop_active:
            self.get_logger().warn("Emergency stop active - ignoring motor command")
            return
        
        self.get_logger().info(f"Received motor command: {msg.command_type}")
        
        if msg.command_type == MotorCommand.MOVE_TO_POSITION:
            self._handle_move_to_position(msg)
        elif msg.command_type == MotorCommand.HOME_MOTORS:
            self._handle_home_motors(msg)
        elif msg.command_type == MotorCommand.STOP_MOTORS:
            self._handle_stop_motors(msg)
        elif msg.command_type == MotorCommand.EMERGENCY_STOP:
            self._handle_emergency_stop(msg)
        else:
            self.get_logger().warn(f"Unknown command type: {msg.command_type}")
    
    def _handle_move_to_position(self, msg: MotorCommand):
        """Handle move to position command"""
        if not self.system_ready:
            self.get_logger().warn("Motor system not ready")
            return
        
        # Check limits
        if msg.check_limits:
            if abs(msg.target_azimuth) > self.max_azimuth:
                self.get_logger().warn(f"Azimuth {msg.target_azimuth}° exceeds limit ±{self.max_azimuth}°")
                return
            
            if abs(msg.target_elevation) > self.max_elevation:
                self.get_logger().warn(f"Elevation {msg.target_elevation}° exceeds limit ±{self.max_elevation}°")
                return
        
        # Execute movement
        with self.movement_lock:
            if self.motor_system:
                # Real motor system
                try:
                    success = self.motor_system.move_to_target(
                        msg.target_azimuth,
                        msg.target_elevation
                    )
                    
                    if success:
                        self.target_position['azimuth'] = msg.target_azimuth
                        self.target_position['elevation'] = msg.target_elevation
                        self.get_logger().info(
                            f"Moving to Az={msg.target_azimuth:.1f}°, El={msg.target_elevation:.1f}°"
                        )
                    else:
                        self.get_logger().error("Motor movement failed")
                        
                except Exception as e:
                    self.get_logger().error(f"Motor movement error: {e}")
            else:
                # Simulation mode
                self.sim_azimuth = msg.target_azimuth
                self.sim_elevation = msg.target_elevation
                self.target_position['azimuth'] = msg.target_azimuth
                self.target_position['elevation'] = msg.target_elevation
                
                self.get_logger().info(
                    f"[SIM] Moved to Az={msg.target_azimuth:.1f}°, El={msg.target_elevation:.1f}°"
                )
    
    def _handle_home_motors(self, msg: MotorCommand):
        """Handle home motors command"""
        self.get_logger().info("Homing motors...")
        
        with self.movement_lock:
            if self.motor_system:
                try:
                    success = self.motor_system.home_all_motors()
                    if success:
                        self.get_logger().info("✅ Motors homed successfully")
                        self.target_position = {'azimuth': 0.0, 'elevation': 0.0}
                    else:
                        self.get_logger().error("❌ Motor homing failed")
                except Exception as e:
                    self.get_logger().error(f"Homing error: {e}")
            else:
                # Simulation mode
                self.sim_azimuth = 0.0
                self.sim_elevation = 0.0
                self.sim_homed = True
                self.target_position = {'azimuth': 0.0, 'elevation': 0.0}
                self.get_logger().info("✅ [SIM] Motors homed")
    
    def _handle_stop_motors(self, msg: MotorCommand):
        """Handle stop motors command"""
        self.get_logger().info("Stopping motors...")
        
        if self.motor_system:
            try:
                self.motor_system.emergency_stop_all()
                self.get_logger().info("✅ Motors stopped")
            except Exception as e:
                self.get_logger().error(f"Stop error: {e}")
        else:
            self.get_logger().info("✅ [SIM] Motors stopped")
    
    def _handle_emergency_stop(self, msg: MotorCommand):
        """Handle emergency stop command"""
        self.get_logger().error("🚨 EMERGENCY STOP ACTIVATED")
        
        self.emergency_stop_active = True
        
        if self.motor_system:
            try:
                self.motor_system.emergency_stop_all()
            except Exception as e:
                self.get_logger().error(f"Emergency stop error: {e}")
    
    def move_to_position_callback(self, goal_handle):
        """Handle MoveToPosition action"""
        self.get_logger().info('Executing MoveToPosition action...')
        
        goal = goal_handle.request
        feedback_msg = MoveToPosition.Feedback()
        result = MoveToPosition.Result()
        
        # Check if emergency stop is active
        if self.emergency_stop_active:
            goal_handle.abort()
            result.success = False
            result.result_message = "Emergency stop active"
            return result
        
        # Start movement
        start_time = time.time()
        target_az = goal.target_azimuth
        target_el = goal.target_elevation
        
        # Execute movement
        with self.movement_lock:
            if self.motor_system:
                try:
                    success = self.motor_system.move_to_target(target_az, target_el)
                except Exception as e:
                    self.get_logger().error(f"Action movement error: {e}")
                    success = False
            else:
                # Simulation
                self.sim_azimuth = target_az
                self.sim_elevation = target_el
                success = True
        
        # Provide feedback and result
        if success:
            current_az, current_el = self.get_current_position()
            
            feedback_msg.current_azimuth = current_az
            feedback_msg.current_elevation = current_el
            feedback_msg.movement_progress = 1.0
            feedback_msg.status_message = "Movement complete"
            goal_handle.publish_feedback(feedback_msg)
            
            goal_handle.succeed()
            result.success = True
            result.final_azimuth = current_az
            result.final_elevation = current_el
            result.movement_time.sec = int(time.time() - start_time)
            result.result_message = "Movement completed successfully"
        else:
            goal_handle.abort()
            result.success = False
            result.result_message = "Movement failed"
        
        return result
    
    def home_motors_callback(self, request, response):
        """Handle home motors service"""
        self.get_logger().info("Home motors service called")
        
        start_time = time.time()
        
        if self.motor_system:
            try:
                success = self.motor_system.home_all_motors()
                response.success = success
                response.azimuth_homed = success
                response.elevation_homed = success
                response.message = "Homing completed" if success else "Homing failed"
            except Exception as e:
                self.get_logger().error(f"Homing service error: {e}")
                response.success = False
                response.message = f"Homing error: {str(e)}"
        else:
            # Simulation
            self.sim_azimuth = 0.0
            self.sim_elevation = 0.0
            self.sim_homed = True
            response.success = True
            response.azimuth_homed = True
            response.elevation_homed = True
            response.message = "[SIM] Homing completed"
        
        response.homing_time.sec = int(time.time() - start_time)
        
        # Update final position
        current_az, current_el = self.get_current_position()
        response.final_position.header.stamp = self.get_clock().now().to_msg()
        response.final_position.azimuth_angle = current_az
        response.final_position.elevation_angle = current_el
        
        return response
    
    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service"""
        self.get_logger().error(f"🚨 Emergency stop service: {request.reason}")
        
        self.emergency_stop_active = True
        
        if self.motor_system:
            try:
                self.motor_system.emergency_stop_all()
                response.success = True
                response.systems_stopped = 1
            except Exception as e:
                response.success = False
                response.systems_stopped = 0
        else:
            response.success = True
            response.systems_stopped = 1  # Simulation
        
        response.message = "Emergency stop executed"
        response.stopped_at = self.get_clock().now().to_msg()
        
        return response
    
    def get_current_position(self) -> tuple:
        """Get current motor position"""
        if self.motor_system:
            try:
                return self.motor_system.get_current_position()
            except:
                return (0.0, 0.0)
        else:
            # Simulation
            return (self.sim_azimuth, self.sim_elevation)
    
    def publish_position(self):
        """Publish current motor position"""
        current_az, current_el = self.get_current_position()
        
        msg = MotorPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        msg.azimuth_angle = current_az
        msg.elevation_angle = current_el
        
        # Set limits
        msg.azimuth_min_angle = -self.max_azimuth
        msg.azimuth_max_angle = self.max_azimuth
        msg.elevation_min_angle = -self.max_elevation
        msg.elevation_max_angle = self.max_elevation
        
        # System status
        msg.system_ready = self.system_ready and not self.emergency_stop_active
        msg.emergency_stop = self.emergency_stop_active
        
        if self.motor_system:
            msg.azimuth_homed = True  # Assume homed for now
            msg.elevation_homed = True
            msg.status_message = "System operational"
        else:
            msg.azimuth_homed = self.sim_homed
            msg.elevation_homed = self.sim_homed
            msg.status_message = "Simulation mode"
        
        self.position_pub.publish(msg)
    
    def publish_status(self):
        """Publish system status"""
        msg = SystemStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.system_name = "Motor Controller"
        msg.version = "1.0.0"
        
        if self.emergency_stop_active:
            msg.system_state = SystemStatus.STATE_EMERGENCY
        elif self.system_ready:
            msg.system_state = SystemStatus.STATE_STANDBY
        else:
            msg.system_state = SystemStatus.STATE_ERROR
        
        msg.motor_system_online = self.system_ready
        
        self.status_pub.publish(msg)
    
    def cleanup(self):
        """Clean up resources"""
        self.get_logger().info("Cleaning up Motor Controller Node...")
        
        if self.motor_system:
            try:
                self.motor_system.cleanup()
            except Exception as e:
                self.get_logger().error(f"Cleanup error: {e}")
        
        self.get_logger().info("✅ Motor Controller cleanup complete")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = MotorControllerNode()
        
        print("🔧 Motor Controller Node started!")
        print("   Subscribing to: /motor_commands")
        print("   Publishing position to: /motor_position")
        print("   Action server: /move_to_position")
        print("   Services: /home_motors, /emergency_stop")
        print("   Press Ctrl+C to stop...")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Motor Controller Node stopped by user")
    except Exception as e:
        print(f"❌ Motor Controller Node error: {e}")
    finally:
        if 'node' in locals():
            node.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 