#!/usr/bin/env python3
"""
Vision Bridge Node for ROS2 Air Defense System
Bridges existing YOLO system with ROS2 ecosystem
"""

import sys
import time
import cv2
import numpy as np
from pathlib import Path
from typing import List, Optional

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS2 message imports
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Custom message imports
from ros2_air_defense.msg import DetectedTarget, TargetList

# Import existing YOLO system
project_root = Path(__file__).parent.parent.parent
sys.path.append(str(project_root))

try:
    from src.vision.yolo_detector import create_yolo_wrapper_from_existing, MultiModelYOLOWrapper
    YOLO_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ YOLO system not available: {e}")
    YOLO_AVAILABLE = False

class VisionBridgeNode(Node):
    """
    ROS2 node that bridges existing YOLO system with ROS2 ecosystem
    Publishes detected targets as ROS2 messages
    """
    
    def __init__(self):
        super().__init__('vision_bridge_node')
        
        # Node parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('target_classes', ['red_balloon', 'blue_balloon'])
        
        # Get parameters
        self.camera_id = self.get_parameter('camera_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.publish_annotated = self.get_parameter('publish_annotated_image').value
        self.target_classes = self.get_parameter('target_classes').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.target_list_pub = self.create_publisher(
            TargetList, 
            'detected_targets', 
            reliable_qos
        )
        
        self.raw_image_pub = self.create_publisher(
            Image, 
            'camera/raw_image', 
            sensor_qos
        )
        
        if self.publish_annotated:
            self.annotated_image_pub = self.create_publisher(
                Image, 
                'camera/annotated_image', 
                sensor_qos
            )
        
        # CV Bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Initialize YOLO system
        self.yolo_wrapper = None
        self.yolo_ready = False
        
        if YOLO_AVAILABLE:
            self._initialize_yolo()
        else:
            self.get_logger().error("YOLO system not available - running in simulation mode")
        
        # Initialize camera
        self.cap = None
        self._initialize_camera()
        
        # State tracking
        self.target_id_counter = 1
        self.detection_count = 0
        self.last_detection_time = time.time()
        
        # Create timer for main processing loop
        timer_period = 1.0 / self.publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.process_frame)
        
        self.get_logger().info(f"Vision Bridge Node initialized")
        self.get_logger().info(f"  Camera ID: {self.camera_id}")
        self.get_logger().info(f"  Publish rate: {self.publish_rate} Hz")
        self.get_logger().info(f"  YOLO available: {self.yolo_ready}")
    
    def _initialize_yolo(self):
        """Initialize YOLO detection system"""
        try:
            self.get_logger().info("Initializing YOLO system...")
            self.yolo_wrapper = create_yolo_wrapper_from_existing()
            
            if self.yolo_wrapper and len(self.yolo_wrapper.get_available_models()) > 0:
                self.yolo_ready = True
                models = self.yolo_wrapper.get_available_models()
                self.get_logger().info(f"✅ YOLO initialized with models: {models}")
                
                # Set confidence threshold
                if hasattr(self.yolo_wrapper, 'active_detector') and self.yolo_wrapper.active_detector:
                    self.yolo_wrapper.active_detector.config.confidence_threshold = self.confidence_threshold
                    self.yolo_wrapper.active_detector.config.target_classes = self.target_classes
                    
            else:
                self.get_logger().error("❌ No YOLO models available")
                
        except Exception as e:
            self.get_logger().error(f"❌ YOLO initialization failed: {e}")
            self.yolo_ready = False
    
    def _initialize_camera(self):
        """Initialize camera system"""
        try:
            self.get_logger().info(f"Initializing camera {self.camera_id}...")
            self.cap = cv2.VideoCapture(self.camera_id)
            
            if self.cap.isOpened():
                # Set camera properties
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                
                self.get_logger().info("✅ Camera initialized successfully")
            else:
                self.get_logger().error("❌ Failed to open camera")
                
        except Exception as e:
            self.get_logger().error(f"❌ Camera initialization failed: {e}")
    
    def process_frame(self):
        """Main processing loop - capture frame and detect targets"""
        if not self.cap or not self.cap.isOpened():
            return
        
        # Capture frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return
        
        current_time = self.get_clock().now()
        
        # Publish raw image
        try:
            raw_image_msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
            raw_image_msg.header.stamp = current_time.to_msg()
            raw_image_msg.header.frame_id = "camera_frame"
            self.raw_image_pub.publish(raw_image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish raw image: {e}")
        
        # Run YOLO detection if available
        target_list = TargetList()
        target_list.header.stamp = current_time.to_msg()
        target_list.header.frame_id = "camera_frame"
        
        annotated_frame = frame.copy()
        
        if self.yolo_ready and self.yolo_wrapper:
            try:
                # Run detection
                detections = self.yolo_wrapper.detect(frame, verbose=False)
                
                # Convert to ROS2 messages
                targets = self._convert_detections_to_ros2(detections, current_time)
                target_list.targets = targets
                
                # Update statistics
                target_list.total_targets = len(targets)
                target_list.enemy_targets = len([t for t in targets if t.is_enemy])
                target_list.friendly_targets = len([t for t in targets if not t.is_enemy])
                target_list.moving_targets = len([t for t in targets if t.target_type == DetectedTarget.TARGET_MOVING])
                target_list.static_targets = len([t for t in targets if t.target_type == DetectedTarget.TARGET_STATIC])
                
                # Find best target
                if targets:
                    best_target = max(targets, key=lambda t: t.priority)
                    target_list.has_priority_target = True
                    target_list.priority_target_id = best_target.target_id
                    target_list.best_target_azimuth = best_target.azimuth
                    target_list.best_target_elevation = best_target.elevation
                    
                    # Calculate average confidence
                    target_list.detection_confidence = sum(t.confidence for t in targets) / len(targets)
                else:
                    target_list.has_priority_target = False
                    target_list.detection_confidence = 0.0
                
                # Update counters
                self.detection_count += len(targets)
                target_list.detection_count = self.detection_count
                
                # Create annotated frame
                if self.publish_annotated:
                    annotated_frame = self._draw_detections(frame, targets)
                
            except Exception as e:
                self.get_logger().error(f"Detection processing failed: {e}")
        
        # Publish target list
        self.target_list_pub.publish(target_list)
        
        # Publish annotated image
        if self.publish_annotated:
            try:
                annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                annotated_image_msg.header.stamp = current_time.to_msg()
                annotated_image_msg.header.frame_id = "camera_frame"
                self.annotated_image_pub.publish(annotated_image_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish annotated image: {e}")
    
    def _convert_detections_to_ros2(self, detections, current_time) -> List[DetectedTarget]:
        """Convert YOLO detections to ROS2 DetectedTarget messages"""
        targets = []
        
        for detection in detections:
            target = DetectedTarget()
            
            # Header
            target.header.stamp = current_time.to_msg()
            target.header.frame_id = "camera_frame"
            
            # Target ID
            target.target_id = self.target_id_counter
            self.target_id_counter += 1
            
            # Basic detection info
            target.class_name = detection.class_name
            target.confidence = detection.confidence
            
            # Pixel coordinates
            target.pixel_x = detection.pixel_x
            target.pixel_y = detection.pixel_y
            
            # Bounding box
            if hasattr(detection, 'bbox') and detection.bbox:
                target.bbox_x1 = detection.bbox[0]
                target.bbox_y1 = detection.bbox[1]
                target.bbox_x2 = detection.bbox[2]
                target.bbox_y2 = detection.bbox[3]
            
            # Estimate motor angles (simplified - coordinate transformer will do this properly)
            # For now, use basic pixel-to-angle conversion
            frame_width, frame_height = 640, 480
            horizontal_fov, vertical_fov = 60.0, 45.0  # degrees
            
            target.azimuth = (detection.pixel_x - frame_width/2) * horizontal_fov / frame_width
            target.elevation = (frame_height/2 - detection.pixel_y) * vertical_fov / frame_height
            
            # Target classification
            target.target_type = DetectedTarget.TARGET_STATIC  # Default, will be updated by coordinate transformer
            
            # Priority and distance
            center_distance = np.sqrt((detection.pixel_x - frame_width/2)**2 + 
                                    (detection.pixel_y - frame_height/2)**2)
            target.distance_to_center = center_distance
            
            # Simple priority calculation
            target.priority = detection.confidence * (1.0 - center_distance / (frame_width/2))
            
            # Enemy/friendly classification
            target.is_enemy = "red" in detection.class_name.lower()
            
            # Tracking info
            target.track_id = getattr(detection, 'track_id', target.target_id)
            target.velocity_x = 0.0  # Will be calculated by tracker
            target.velocity_y = 0.0
            
            # Engagement status
            target.is_engaged = False
            target.is_locked = False
            
            targets.append(target)
        
        return targets
    
    def _draw_detections(self, frame: np.ndarray, targets: List[DetectedTarget]) -> np.ndarray:
        """Draw detection annotations on frame"""
        annotated_frame = frame.copy()
        
        for target in targets:
            # Color based on enemy/friendly
            if target.is_enemy:
                color = (0, 0, 255)  # Red for enemy
                label = "DÜŞMAN"
            else:
                color = (255, 0, 0)  # Blue for friendly
                label = "DOST"
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, 
                         (target.bbox_x1, target.bbox_y1), 
                         (target.bbox_x2, target.bbox_y2), 
                         color, 2)
            
            # Draw center point
            center = (int(target.pixel_x), int(target.pixel_y))
            cv2.circle(annotated_frame, center, 5, color, -1)
            
            # Draw crosshair
            cv2.line(annotated_frame, (center[0]-15, center[1]), (center[0]+15, center[1]), color, 2)
            cv2.line(annotated_frame, (center[0], center[1]-15), (center[0], center[1]+15), color, 2)
            
            # Label with info
            label_text = f"{label} ID:{target.target_id} C:{target.confidence:.2f}"
            angle_text = f"Az:{target.azimuth:.1f}° El:{target.elevation:.1f}°"
            
            # Text background
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 1
            
            label_size = cv2.getTextSize(label_text, font, font_scale, thickness)[0]
            angle_size = cv2.getTextSize(angle_text, font, font_scale, thickness)[0]
            
            max_width = max(label_size[0], angle_size[0])
            total_height = label_size[1] + angle_size[1] + 10
            
            cv2.rectangle(annotated_frame,
                         (target.bbox_x1, target.bbox_y1 - total_height - 5),
                         (target.bbox_x1 + max_width + 5, target.bbox_y1),
                         color, -1)
            
            # Text
            cv2.putText(annotated_frame, label_text,
                       (target.bbox_x1 + 2, target.bbox_y1 - angle_size[1] - 8),
                       font, font_scale, (255, 255, 255), thickness)
            
            cv2.putText(annotated_frame, angle_text,
                       (target.bbox_x1 + 2, target.bbox_y1 - 2),
                       font, font_scale, (255, 255, 255), thickness)
        
        # Add system info
        info_text = f"ROS2 Vision Bridge - Targets: {len(targets)} | Rate: {self.publish_rate}Hz"
        cv2.putText(annotated_frame, info_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return annotated_frame
    
    def cleanup(self):
        """Clean up resources"""
        self.get_logger().info("Cleaning up Vision Bridge Node...")
        
        if self.cap:
            self.cap.release()
        
        self.get_logger().info("✅ Vision Bridge Node cleanup complete")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = VisionBridgeNode()
        
        print("🤖 Vision Bridge Node started!")
        print("   Publishing detected targets to: /detected_targets")
        print("   Publishing raw images to: /camera/raw_image") 
        print("   Publishing annotated images to: /camera/annotated_image")
        print("   Press Ctrl+C to stop...")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Vision Bridge Node stopped by user")
    except Exception as e:
        print(f"❌ Vision Bridge Node error: {e}")
    finally:
        if 'node' in locals():
            node.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 