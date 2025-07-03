#!/usr/bin/env python3
"""
Coordinate Transformation System for Air Defense
Converts pixel coordinates to motor angles with support for both
static targets (batch processing) and moving targets (real-time)
"""

import math
import time
import json
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
from enum import Enum

class TargetType(Enum):
    STATIC = "static"
    MOVING = "moving"

@dataclass
class DetectedTarget:
    """Represents a detected target with all necessary information"""
    pixel_x: float
    pixel_y: float
    confidence: float
    class_name: str  # 'red_balloon' or 'blue_balloon'
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2)
    target_type: TargetType
    timestamp: float
    track_id: Optional[int] = None  # For moving target tracking

@dataclass
class MotorTarget:
    """Motor command target with angle coordinates"""
    azimuth: float
    elevation: float
    original_pixel: Tuple[float, float]
    confidence: float
    target_type: TargetType
    priority: int
    timestamp: float

class CameraCalibration:
    """Camera calibration parameters and conversion methods"""
    
    def __init__(self, config_file: str = None):
        """Initialize camera calibration parameters"""
        
        # Default calibration values
        default_config = {
            "camera": {
                "horizontal_fov": 60.0,  # degrees
                "vertical_fov": 45.0,    # degrees
                "image_width": 640,
                "image_height": 480,
                "lens_distortion": {
                    "k1": 0.0,
                    "k2": 0.0,
                    "p1": 0.0,
                    "p2": 0.0
                }
            },
            "mount_offset": {
                "azimuth_offset": 0.0,   # Kamera-namlu hizalama farkı
                "elevation_offset": 0.0,
                "distance_offset": 0.0   # Kamera-namlu mesafesi (parallax)
            }
        }
        
        if config_file:
            with open(config_file, 'r') as f:
                config = json.load(f)
        else:
            config = default_config
        
        # Camera parameters
        self.h_fov = config["camera"]["horizontal_fov"]
        self.v_fov = config["camera"]["vertical_fov"]
        self.img_width = config["camera"]["image_width"]
        self.img_height = config["camera"]["image_height"]
        
        # Calculate center coordinates
        self.center_x = self.img_width / 2.0
        self.center_y = self.img_height / 2.0
        
        # Mount offsets
        self.azimuth_offset = config["mount_offset"]["azimuth_offset"]
        self.elevation_offset = config["mount_offset"]["elevation_offset"]
        
        # Calculate degrees per pixel
        self.deg_per_pixel_h = self.h_fov / self.img_width
        self.deg_per_pixel_v = self.v_fov / self.img_height
        
        print(f"Camera calibration initialized:")
        print(f"  FOV: {self.h_fov}° x {self.v_fov}°")
        print(f"  Resolution: {self.img_width} x {self.img_height}")
        print(f"  Deg/pixel: {self.deg_per_pixel_h:.4f}° x {self.deg_per_pixel_v:.4f}°")
    
    def pixel_to_angle(self, pixel_x: float, pixel_y: float) -> Tuple[float, float]:
        """
        Convert pixel coordinates to motor angles
        
        Args:
            pixel_x: X coordinate in pixels (0 = left edge)
            pixel_y: Y coordinate in pixels (0 = top edge)
        
        Returns:
            Tuple[azimuth, elevation] in degrees
        """
        # Calculate offset from center
        dx = pixel_x - self.center_x
        dy = pixel_y - self.center_y
        
        # Convert to angles
        azimuth_raw = dx * self.deg_per_pixel_h
        elevation_raw = -dy * self.deg_per_pixel_v  # Negative because Y is inverted
        
        # Apply mount offsets
        azimuth = azimuth_raw + self.azimuth_offset
        elevation = elevation_raw + self.elevation_offset
        
        return azimuth, elevation
    
    def angle_to_pixel(self, azimuth: float, elevation: float) -> Tuple[float, float]:
        """
        Convert motor angles back to pixel coordinates (for verification)
        """
        # Remove mount offsets
        azimuth_raw = azimuth - self.azimuth_offset
        elevation_raw = elevation - self.elevation_offset
        
        # Convert to pixels
        dx = azimuth_raw / self.deg_per_pixel_h
        dy = -elevation_raw / self.deg_per_pixel_v
        
        # Calculate absolute pixel coordinates
        pixel_x = self.center_x + dx
        pixel_y = self.center_y + dy
        
        return pixel_x, pixel_y


class TargetClassifier:
    """Classifies targets as static or moving based on tracking data"""
    
    def __init__(self, movement_threshold: float = 5.0, time_window: float = 0.5):
        """
        Initialize target classifier
        
        Args:
            movement_threshold: Pixel movement threshold to consider target as moving
            time_window: Time window to analyze movement (seconds)
        """
        self.movement_threshold = movement_threshold
        self.time_window = time_window
        self.target_history: Dict[int, List[Tuple[float, float, float]]] = {}  # track_id: [(x, y, timestamp)]
    
    def classify_target(self, track_id: int, pixel_x: float, pixel_y: float, timestamp: float) -> TargetType:
        """
        Classify target as static or moving based on recent movement
        
        Args:
            track_id: Unique identifier for the target
            pixel_x, pixel_y: Current position
            timestamp: Current time
        
        Returns:
            TargetType.STATIC or TargetType.MOVING
        """
        # Add current position to history
        if track_id not in self.target_history:
            self.target_history[track_id] = []
        
        self.target_history[track_id].append((pixel_x, pixel_y, timestamp))
        
        # Remove old entries outside time window
        cutoff_time = timestamp - self.time_window
        self.target_history[track_id] = [
            entry for entry in self.target_history[track_id] 
            if entry[2] >= cutoff_time
        ]
        
        # Need at least 2 points to determine movement
        if len(self.target_history[track_id]) < 2:
            return TargetType.STATIC
        
        # Calculate maximum movement in time window
        positions = self.target_history[track_id]
        max_movement = 0.0
        
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            movement = math.sqrt(dx*dx + dy*dy)
            max_movement = max(max_movement, movement)
        
        # Classify based on movement threshold
        if max_movement > self.movement_threshold:
            return TargetType.MOVING
        else:
            return TargetType.STATIC


class CoordinateTransformer:
    """Main coordinate transformation system with hybrid mode support"""
    
    def __init__(self, config_file: str = None):
        """Initialize coordinate transformer"""
        self.calibration = CameraCalibration(config_file)
        self.classifier = TargetClassifier()
        
        # Target management
        self.static_targets: List[MotorTarget] = []
        self.moving_targets: List[MotorTarget] = []
        self.processed_targets: List[int] = []  # Track IDs of already processed targets
        
    def process_detections(self, detections: List[DetectedTarget]) -> Tuple[List[MotorTarget], List[MotorTarget]]:
        """
        Process YOLO detections and separate into static and moving targets
        
        Args:
            detections: List of detected targets from YOLO
        
        Returns:
            Tuple[static_targets, moving_targets]
        """
        current_time = time.time()
        new_static = []
        new_moving = []
        
        for detection in detections:
            # Skip friendly targets (blue balloons)
            if detection.class_name == 'blue_balloon':
                continue
            
            # Skip if already processed (for static targets)
            if detection.track_id and detection.track_id in self.processed_targets:
                continue
            
            # Classify target type
            if detection.track_id:
                target_type = self.classifier.classify_target(
                    detection.track_id, 
                    detection.pixel_x, 
                    detection.pixel_y, 
                    current_time
                )
            else:
                target_type = TargetType.STATIC  # Default for untracked detections
            
            # Convert to motor coordinates
            azimuth, elevation = self.calibration.pixel_to_angle(
                detection.pixel_x, 
                detection.pixel_y
            )
            
            # Create motor target
            motor_target = MotorTarget(
                azimuth=azimuth,
                elevation=elevation,
                original_pixel=(detection.pixel_x, detection.pixel_y),
                confidence=detection.confidence,
                target_type=target_type,
                priority=self._calculate_priority(detection, azimuth, elevation),
                timestamp=current_time
            )
            
            # Add to appropriate list
            if target_type == TargetType.STATIC:
                new_static.append(motor_target)
                if detection.track_id:
                    self.processed_targets.append(detection.track_id)
            else:
                new_moving.append(motor_target)
        
        return new_static, new_moving
    
    def _calculate_priority(self, detection: DetectedTarget, azimuth: float, elevation: float) -> int:
        """
        Calculate target priority based on various factors
        
        Priority factors:
        1. Confidence score (higher = better)
        2. Distance from current position (closer = better)
        3. Target size (larger = easier)
        4. Target type (moving targets get higher priority)
        
        Returns:
            Priority score (higher = more important)
        """
        priority = 0
        
        # Confidence factor (0-100)
        priority += int(detection.confidence * 100)
        
        # Distance factor (closer targets preferred)
        angle_distance = math.sqrt(azimuth*azimuth + elevation*elevation)
        distance_factor = max(0, 100 - int(angle_distance))
        priority += distance_factor
        
        # Size factor (larger targets easier to hit)
        bbox_width = detection.bbox[2] - detection.bbox[0]
        bbox_height = detection.bbox[3] - detection.bbox[1]
        size_factor = min(50, int((bbox_width * bbox_height) / 100))
        priority += size_factor
        
        # Target type factor
        if detection.target_type == TargetType.MOVING:
            priority += 50  # Moving targets get bonus
        
        return priority
    
    def plan_static_shooting_sequence(self, static_targets: List[MotorTarget]) -> List[MotorTarget]:
        """
        Plan optimal shooting sequence for static targets
        
        Args:
            static_targets: List of static motor targets
        
        Returns:
            Optimally ordered list of targets
        """
        if not static_targets:
            return []
        
        # Sort by priority first
        targets_by_priority = sorted(static_targets, key=lambda t: t.priority, reverse=True)
        
        # Then optimize path to minimize total movement
        optimized_sequence = self._optimize_shooting_path(targets_by_priority)
        
        return optimized_sequence
    
    def _optimize_shooting_path(self, targets: List[MotorTarget]) -> List[MotorTarget]:
        """
        Optimize shooting path using nearest neighbor algorithm
        """
        if len(targets) <= 1:
            return targets
        
        # Start from home position (0, 0)
        current_pos = (0.0, 0.0)
        remaining_targets = targets.copy()
        optimized_sequence = []
        
        while remaining_targets:
            # Find nearest target
            min_distance = float('inf')
            nearest_target = None
            nearest_index = -1
            
            for i, target in enumerate(remaining_targets):
                distance = math.sqrt(
                    (target.azimuth - current_pos[0])**2 + 
                    (target.elevation - current_pos[1])**2
                )
                if distance < min_distance:
                    min_distance = distance
                    nearest_target = target
                    nearest_index = i
            
            # Add nearest target to sequence
            optimized_sequence.append(nearest_target)
            current_pos = (nearest_target.azimuth, nearest_target.elevation)
            remaining_targets.pop(nearest_index)
        
        return optimized_sequence
    
    def get_best_moving_target(self, moving_targets: List[MotorTarget]) -> Optional[MotorTarget]:
        """
        Get the best moving target for immediate engagement
        
        Args:
            moving_targets: List of moving targets
        
        Returns:
            Best target to engage or None
        """
        if not moving_targets:
            return None
        
        # Sort by priority and return the best one
        best_target = max(moving_targets, key=lambda t: t.priority)
        return best_target
    
    def update_camera_calibration(self, new_fov_h: float = None, new_fov_v: float = None):
        """Update camera calibration parameters"""
        if new_fov_h:
            self.calibration.h_fov = new_fov_h
            self.calibration.deg_per_pixel_h = new_fov_h / self.calibration.img_width
        
        if new_fov_v:
            self.calibration.v_fov = new_fov_v
            self.calibration.deg_per_pixel_v = new_fov_v / self.calibration.img_height
        
        print(f"Camera calibration updated: {self.calibration.h_fov}° x {self.calibration.v_fov}°")


if __name__ == "__main__":
    # Test script
    transformer = CoordinateTransformer()
    
    # Test pixel to angle conversion
    test_pixels = [(320, 240), (0, 0), (640, 480), (160, 120)]
    
    print("Testing pixel to angle conversion:")
    for px, py in test_pixels:
        az, el = transformer.calibration.pixel_to_angle(px, py)
        print(f"Pixel ({px}, {py}) → Motor ({az:.2f}°, {el:.2f}°)")
    
    # Test with mock detections
    mock_detections = [
        DetectedTarget(300, 200, 0.9, 'red_balloon', (280, 180, 320, 220), TargetType.STATIC, time.time(), 1),
        DetectedTarget(400, 300, 0.8, 'red_balloon', (380, 280, 420, 320), TargetType.STATIC, time.time(), 2),
    ]
    
    static, moving = transformer.process_detections(mock_detections)
    print(f"\nProcessed {len(static)} static targets, {len(moving)} moving targets") 