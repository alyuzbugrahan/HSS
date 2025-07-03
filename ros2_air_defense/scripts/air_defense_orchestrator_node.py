#!/usr/bin/env python3
"""
Air Defense Orchestrator Node for ROS2 Air Defense System
Main coordination node implementing "Scan-Plan-Execute" strategy
"""

import time
import threading
from enum import Enum
from typing import List, Dict, Optional

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Custom message imports
from ros2_air_defense.msg import (
    DetectedTarget, TargetList, MotorPosition, MotorCommand, 
    FiringCommand, SystemStatus, SafetyStatus
)
from ros2_air_defense.action import EngageTarget, MoveToPosition, ScanArea
from ros2_air_defense.srv import ArmSystem, DisarmSystem, EmergencyStop, SetTargetingMode

class SystemState(Enum):
    """System operational states"""
    OFFLINE = 0
    INITIALIZING = 1
    STANDBY = 2
    ARMED = 3
    SCANNING = 4
    ENGAGING = 5
    EMERGENCY = 6

class TargetingMode(Enum):
    """Targeting modes"""
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    SEMI_AUTONOMOUS = "semi_autonomous"

class AirDefenseOrchestratorNode(Node):
    """
    Main orchestrator node that coordinates all air defense subsystems
    Implements Scan-Plan-Execute autonomous targeting strategy
    """
    
    def __init__(self):
        super().__init__('air_defense_orchestrator_node')
        
        # Node parameters
        self.declare_parameter('targeting_mode', 'manual')
        self.declare_parameter('auto_engagement_enabled', False)
        self.declare_parameter('scan_on_startup', True)
        self.declare_parameter('engagement_timeout', 30.0)  # seconds
        self.declare_parameter('priority_threshold', 0.7)
        self.declare_parameter('max_engagement_distance', 150.0)  # degrees from center
        
        # Get parameters
        mode_str = self.get_parameter('targeting_mode').value
        self.auto_engagement = self.get_parameter('auto_engagement_enabled').value
        self.scan_on_startup = self.get_parameter('scan_on_startup').value
        self.engagement_timeout = self.get_parameter('engagement_timeout').value
        self.priority_threshold = self.get_parameter('priority_threshold').value
        self.max_engagement_distance = self.get_parameter('max_engagement_distance').value
        
        # Parse targeting mode
        try:
            self.targeting_mode = TargetingMode(mode_str)
        except ValueError:
            self.targeting_mode = TargetingMode.MANUAL
            self.get_logger().warn(f"Invalid targeting mode '{mode_str}', using MANUAL")
        
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
        self.motor_command_pub = self.create_publisher(
            MotorCommand,
            'motor_commands',
            reliable_qos
        )
        
        self.firing_command_pub = self.create_publisher(
            FiringCommand,
            'firing_commands',
            reliable_qos
        )
        
        self.system_status_pub = self.create_publisher(
            SystemStatus,
            'air_defense_status',
            reliable_qos
        )
        
        # Subscribers
        self.target_list_sub = self.create_subscription(
            TargetList,
            'processed_targets',
            self.target_list_callback,
            reliable_qos
        )
        
        self.motor_position_sub = self.create_subscription(
            MotorPosition,
            'motor_position',
            self.motor_position_callback,
            fast_qos
        )
        
        self.safety_status_sub = self.create_subscription(
            SafetyStatus,
            'safety_status',
            self.safety_status_callback,
            reliable_qos
        )
        
        # Action clients
        self.engage_target_client = ActionClient(self, EngageTarget, 'engage_target')
        self.move_to_position_client = ActionClient(self, MoveToPosition, 'move_to_position')
        self.scan_area_client = ActionClient(self, ScanArea, 'scan_area')
        
        # Service clients
        self.arm_system_client = self.create_client(ArmSystem, 'arm_firing_system')
        self.disarm_system_client = self.create_client(DisarmSystem, 'disarm_firing_system')
        self.emergency_stop_client = self.create_client(EmergencyStop, 'emergency_stop_firing')
        
        # Service servers
        self.set_mode_service = self.create_service(
            SetTargetingMode,
            'set_targeting_mode',
            self.set_targeting_mode_callback
        )
        
        # System state
        self.system_state = SystemState.INITIALIZING
        self.armed = False
        self.emergency_stop_active = False
        
        # Target management
        self.active_targets: Dict[int, DetectedTarget] = {}
        self.priority_target: Optional[DetectedTarget] = None
        self.engagement_queue: List[DetectedTarget] = []
        self.current_engagement: Optional[int] = None  # target_id
        
        # Subsystem status
        self.motor_position = MotorPosition()
        self.safety_status = SafetyStatus()
        self.subsystem_status = {
            'vision': False,
            'motors': False,
            'firing': False,
            'coordinator': False
        }
        
        # Statistics
        self.total_targets_detected = 0
        self.total_engagements = 0
        self.successful_engagements = 0
        self.start_time = time.time()
        
        # Threading
        self.state_lock = threading.Lock()
        
        # Timers
        self.status_timer = self.create_timer(1.0, self.publish_system_status)
        self.strategy_timer = self.create_timer(0.5, self.execute_strategy)
        
        # Initialize system
        self.create_timer(2.0, self.initialize_system, clock=self.get_clock())
        
        self.get_logger().info(f"Air Defense Orchestrator Node initialized")
        self.get_logger().info(f"  Targeting mode: {self.targeting_mode.value}")
        self.get_logger().info(f"  Auto engagement: {self.auto_engagement}")
        self.get_logger().info(f"  Scan on startup: {self.scan_on_startup}")
    
    def initialize_system(self):
        """Initialize the air defense system"""
        self.get_logger().info("🚀 Initializing Air Defense System...")
        
        # Wait for subsystems to come online
        self.create_timer(1.0, self.check_subsystem_status)
        
        if self.scan_on_startup and self.targeting_mode == TargetingMode.AUTONOMOUS:
            self.get_logger().info("Will perform initial area scan after initialization")
    
    def check_subsystem_status(self):
        """Check if all subsystems are ready"""
        # This would check actual subsystem status
        # For now, assume they're ready after a delay
        
        with self.state_lock:
            self.subsystem_status['vision'] = True
            self.subsystem_status['motors'] = True
            self.subsystem_status['firing'] = True
            self.subsystem_status['coordinator'] = True
            
            if all(self.subsystem_status.values()):
                self.system_state = SystemState.STANDBY
                self.get_logger().info("✅ All subsystems online - System ready")
                
                if self.scan_on_startup and self.targeting_mode == TargetingMode.AUTONOMOUS:
                    self.create_timer(2.0, lambda: self.start_area_scan(), clock=self.get_clock())
    
    def target_list_callback(self, msg: TargetList):
        """Process incoming target list"""
        with self.state_lock:
            # Update target tracking
            current_target_ids = set()
            
            for target in msg.targets:
                current_target_ids.add(target.target_id)
                self.active_targets[target.target_id] = target
            
            # Remove lost targets
            lost_targets = set(self.active_targets.keys()) - current_target_ids
            for target_id in lost_targets:
                if target_id in self.active_targets:
                    del self.active_targets[target_id]
                    self.get_logger().debug(f"Lost track of target {target_id}")
            
            # Update priority target
            if msg.has_priority_target and msg.priority_target_id in self.active_targets:
                self.priority_target = self.active_targets[msg.priority_target_id]
            else:
                self.priority_target = None
            
            # Update statistics
            self.total_targets_detected = max(self.total_targets_detected, msg.detection_count)
    
    def motor_position_callback(self, msg: MotorPosition):
        """Update motor position"""
        self.motor_position = msg
        self.subsystem_status['motors'] = msg.system_ready
    
    def safety_status_callback(self, msg: SafetyStatus):
        """Update safety status"""
        self.safety_status = msg
        
        # Check for emergency conditions
        if msg.safety_level >= SafetyStatus.SAFETY_EMERGENCY:
            if not self.emergency_stop_active:
                self.get_logger().error("🚨 Safety emergency detected - activating emergency stop")
                self.trigger_emergency_stop("Safety system emergency")
    
    def execute_strategy(self):
        """Main strategy execution - Scan-Plan-Execute"""
        if self.system_state == SystemState.STANDBY:
            # In standby, monitor for targets but don't engage
            if self.targeting_mode == TargetingMode.AUTONOMOUS and self.auto_engagement:
                if self.priority_target and self.priority_target.is_enemy:
                    if self.priority_target.priority >= self.priority_threshold:
                        self.plan_engagement()
        
        elif self.system_state == SystemState.ARMED:
            # Armed state - ready to engage
            if self.targeting_mode == TargetingMode.AUTONOMOUS:
                if self.priority_target and self.priority_target.is_enemy:
                    if self.priority_target.priority >= self.priority_threshold:
                        self.execute_engagement()
            
        elif self.system_state == SystemState.ENGAGING:
            # Monitor ongoing engagement
            self.monitor_engagement()
    
    def plan_engagement(self):
        """Plan target engagement strategy"""
        if not self.priority_target:
            return
        
        target = self.priority_target
        
        # Check if target is within engagement parameters
        if target.distance_to_center > self.max_engagement_distance:
            self.get_logger().debug(f"Target {target.target_id} too far from center for engagement")
            return
        
        # Check safety conditions
        if self.safety_status.safety_level > SafetyStatus.SAFETY_WARNING:
            self.get_logger().warn("Safety conditions not met for engagement")
            return
        
        # Add to engagement queue if not already there
        if target.target_id not in [t.target_id for t in self.engagement_queue]:
            self.engagement_queue.append(target)
            self.get_logger().info(f"Target {target.target_id} added to engagement queue")
    
    def execute_engagement(self):
        """Execute target engagement"""
        if not self.engagement_queue or self.current_engagement is not None:
            return
        
        target = self.engagement_queue.pop(0)
        
        self.get_logger().info(f"🎯 Engaging target {target.target_id}")
        
        with self.state_lock:
            self.system_state = SystemState.ENGAGING
            self.current_engagement = target.target_id
        
        # Send engagement action
        goal = EngageTarget.Goal()
        goal.target_id = target.target_id
        goal.target_azimuth = target.azimuth
        goal.target_elevation = target.elevation
        goal.target_class = target.class_name
        goal.engagement_mode = EngageTarget.Goal.ENGAGE_SINGLE_SHOT
        goal.confirm_before_firing = False  # Auto mode
        goal.max_engagement_time.sec = int(self.engagement_timeout)
        
        future = self.engage_target_client.send_goal_async(goal)
        future.add_done_callback(self.engagement_response_callback)
    
    def engagement_response_callback(self, future):
        """Handle engagement action response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Engagement goal was rejected")
            self.engagement_completed(success=False)
            return
        
        self.get_logger().info("Engagement goal accepted")
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.engagement_result_callback)
    
    def engagement_result_callback(self, future):
        """Handle engagement action result"""
        result = future.result().result
        
        success = result.success
        target_hit = result.target_hit
        shots_fired = result.shots_fired
        
        if success:
            self.get_logger().info(f"✅ Engagement completed: {shots_fired} shots fired, target hit: {target_hit}")
            self.successful_engagements += 1
        else:
            self.get_logger().warn(f"❌ Engagement failed: {result.result_message}")
        
        self.total_engagements += 1
        self.engagement_completed(success)
    
    def engagement_completed(self, success: bool):
        """Clean up after engagement completion"""
        with self.state_lock:
            self.current_engagement = None
            
            # Return to appropriate state
            if self.armed:
                self.system_state = SystemState.ARMED
            else:
                self.system_state = SystemState.STANDBY
        
        self.get_logger().info("Engagement cycle completed")
    
    def monitor_engagement(self):
        """Monitor ongoing engagement"""
        # This would monitor the engagement progress
        # For now, just check if we're still in engagement state
        pass
    
    def start_area_scan(self):
        """Start autonomous area scanning"""
        if self.system_state != SystemState.STANDBY:
            return
        
        self.get_logger().info("🔍 Starting autonomous area scan")
        
        with self.state_lock:
            self.system_state = SystemState.SCANNING
        
        # Send scan action
        goal = ScanArea.Goal()
        goal.scan_azimuth_min = -90.0
        goal.scan_azimuth_max = 90.0
        goal.scan_elevation_min = -45.0
        goal.scan_elevation_max = 45.0
        goal.scan_pattern = ScanArea.Goal.PATTERN_RASTER
        goal.scan_speed = 0.5
        goal.max_scan_time = 60  # 1 minute max
        goal.return_to_start = True
        goal.dwell_time = 1.0
        
        future = self.scan_area_client.send_goal_async(goal)
        future.add_done_callback(self.scan_response_callback)
    
    def scan_response_callback(self, future):
        """Handle scan action response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Scan goal was rejected")
            with self.state_lock:
                self.system_state = SystemState.STANDBY
            return
        
        self.get_logger().info("Area scan started")
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.scan_result_callback)
    
    def scan_result_callback(self, future):
        """Handle scan action result"""
        result = future.result().result
        
        targets_found = result.targets_found
        enemy_targets = result.enemy_targets_found
        
        self.get_logger().info(f"✅ Area scan completed: {targets_found} targets found ({enemy_targets} enemies)")
        
        with self.state_lock:
            self.system_state = SystemState.STANDBY
    
    def set_targeting_mode_callback(self, request, response):
        """Handle set targeting mode service"""
        try:
            new_mode = TargetingMode(request.targeting_mode)
            old_mode = self.targeting_mode
            
            self.targeting_mode = new_mode
            
            response.success = True
            response.previous_mode = old_mode.value
            response.current_mode = new_mode.value
            response.message = f"Targeting mode changed from {old_mode.value} to {new_mode.value}"
            response.active_targets = len(self.active_targets)
            response.mode_change_allowed = True
            
            self.get_logger().info(f"Targeting mode changed to {new_mode.value}")
            
        except ValueError:
            response.success = False
            response.current_mode = self.targeting_mode.value
            response.message = f"Invalid targeting mode: {request.targeting_mode}"
        
        return response
    
    def trigger_emergency_stop(self, reason: str):
        """Trigger emergency stop"""
        self.get_logger().error(f"🚨 EMERGENCY STOP: {reason}")
        
        with self.state_lock:
            self.emergency_stop_active = True
            self.system_state = SystemState.EMERGENCY
            self.armed = False
            self.current_engagement = None
            self.engagement_queue.clear()
        
        # Send emergency stop to all subsystems
        if self.emergency_stop_client.service_is_ready():
            request = EmergencyStop.Request()
            request.reason = reason
            request.operator_id = "orchestrator"
            request.stop_all_systems = True
            
            future = self.emergency_stop_client.call_async(request)
    
    def publish_system_status(self):
        """Publish overall system status"""
        msg = SystemStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.system_name = "Air Defense System"
        msg.version = "1.0.0"
        msg.uptime_seconds = int(time.time() - self.start_time)
        
        # Map internal state to message state
        state_mapping = {
            SystemState.OFFLINE: SystemStatus.STATE_OFFLINE,
            SystemState.INITIALIZING: SystemStatus.STATE_INITIALIZING,
            SystemState.STANDBY: SystemStatus.STATE_STANDBY,
            SystemState.ARMED: SystemStatus.STATE_ARMED,
            SystemState.SCANNING: SystemStatus.STATE_STANDBY,  # Scanning is part of standby
            SystemState.ENGAGING: SystemStatus.STATE_ENGAGING,
            SystemState.EMERGENCY: SystemStatus.STATE_EMERGENCY
        }
        
        msg.system_state = state_mapping.get(self.system_state, SystemStatus.STATE_ERROR)
        
        # Subsystem status
        msg.vision_system_online = self.subsystem_status['vision']
        msg.motor_system_online = self.subsystem_status['motors']
        msg.firing_system_online = self.subsystem_status['firing']
        msg.coordinate_system_online = self.subsystem_status['coordinator']
        
        # Statistics
        msg.total_targets_detected = self.total_targets_detected
        msg.successful_engagements = self.successful_engagements
        msg.failed_engagements = self.total_engagements - self.successful_engagements
        
        # Current operation
        msg.active_targets = len(self.active_targets)
        msg.engaged_targets = 1 if self.current_engagement else 0
        msg.current_mode = self.targeting_mode.value
        
        if self.system_state == SystemState.SCANNING:
            msg.active_mission = "Area Scan"
        elif self.system_state == SystemState.ENGAGING:
            msg.active_mission = f"Engaging Target {self.current_engagement}"
        else:
            msg.active_mission = "Standby"
        
        # System health
        msg.safety_systems_ok = (self.safety_status.safety_level <= SafetyStatus.SAFETY_WARNING)
        
        self.system_status_pub.publish(msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = AirDefenseOrchestratorNode()
        
        print("🎯 Air Defense Orchestrator Node started!")
        print("   Main coordination and strategy execution")
        print("   Implementing Scan-Plan-Execute autonomous targeting")
        print("   Publishing system status to: /air_defense_status")
        print("   Press Ctrl+C to stop...")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Air Defense Orchestrator stopped by user")
    except Exception as e:
        print(f"❌ Air Defense Orchestrator error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 