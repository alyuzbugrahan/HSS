#!/usr/bin/env python3
"""
YOLO Detector Wrapper for Air Defense System
Converts existing YOLO detection results to our DetectedTarget format
Compatible with both Computer_Vision and Arayüz systems
"""

import time
import cv2
import numpy as np
from typing import List, Optional, Dict, Any
from ultralytics import YOLO
from dataclasses import dataclass

# Import our coordinate system
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))

from motors.coordinate_transformer import DetectedTarget, TargetType

@dataclass
class YOLOConfig:
    """Configuration for YOLO detector"""
    model_path: str
    confidence_threshold: float = 0.7
    nms_threshold: float = 0.4
    target_classes: List[str] = None  # ['red_balloon', 'blue_balloon'] or None for all
    max_detections: int = 20

class YOLODetectorWrapper:
    """
    Wrapper for YOLO detector that integrates with our targeting system
    Compatible with existing Computer_Vision models
    """
    
    def __init__(self, config: YOLOConfig):
        """Initialize YOLO detector wrapper"""
        self.config = config
        self.model = None
        self.is_loaded = False
        self.detection_count = 0
        self.track_id_counter = 1
        
        # Class name mapping for different missions
        self.class_mappings = {
            'balloon': 'red_balloon',  # Mission1 format
            'red_balloon': 'red_balloon',
            'blue_balloon': 'blue_balloon',
            'kirmizi_balon': 'red_balloon',
            'mavi_balon': 'blue_balloon'
        }
        
        print(f"YOLO Wrapper initialized with model: {config.model_path}")
        
    def load_model(self) -> bool:
        """Load YOLO model"""
        try:
            self.model = YOLO(self.config.model_path)
            self.is_loaded = True
            print(f"✅ YOLO model loaded successfully: {self.config.model_path}")
            
            # Print model info
            if hasattr(self.model, 'names'):
                print(f"Model classes: {list(self.model.names.values())}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to load YOLO model: {e}")
            self.is_loaded = False
            return False
    
    def detect(self, frame: np.ndarray, verbose: bool = False) -> List[DetectedTarget]:
        """
        Run YOLO detection on frame and convert to DetectedTarget format
        
        Args:
            frame: Input image (BGR format)
            verbose: Print detection details
            
        Returns:
            List of DetectedTarget objects
        """
        if not self.is_loaded:
            if not self.load_model():
                return []
        
        try:
            # Run YOLO inference
            results = self.model.predict(
                source=frame,
                conf=self.config.confidence_threshold,
                iou=self.config.nms_threshold,
                verbose=False,
                max_det=self.config.max_detections
            )
            
            # Convert to our format
            detected_targets = self._convert_yolo_results(results[0], verbose)
            
            if verbose and detected_targets:
                print(f"🎯 Detected {len(detected_targets)} targets")
                for target in detected_targets:
                    print(f"  - {target.class_name}: {target.confidence:.2f} at ({target.pixel_x:.0f}, {target.pixel_y:.0f})")
            
            return detected_targets
            
        except Exception as e:
            print(f"❌ Detection error: {e}")
            return []
    
    def _convert_yolo_results(self, yolo_result, verbose: bool = False) -> List[DetectedTarget]:
        """Convert YOLO results to DetectedTarget format"""
        detected_targets = []
        current_time = time.time()
        
        if not hasattr(yolo_result, 'boxes') or yolo_result.boxes is None:
            return detected_targets
        
        for i, box in enumerate(yolo_result.boxes):
            try:
                # Extract box information
                xyxy = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
                confidence = float(box.conf[0].cpu().numpy())
                class_id = int(box.cls[0].cpu().numpy())
                
                # Get class name
                if hasattr(yolo_result, 'names') and class_id in yolo_result.names:
                    raw_class_name = yolo_result.names[class_id]
                else:
                    raw_class_name = f"class_{class_id}"
                
                # Map to our class names
                mapped_class_name = self.class_mappings.get(raw_class_name, raw_class_name)
                
                # Filter by target classes if specified
                if self.config.target_classes and mapped_class_name not in self.config.target_classes:
                    continue
                
                # Calculate center coordinates
                x1, y1, x2, y2 = xyxy
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
                
                # Create DetectedTarget
                detected_target = DetectedTarget(
                    pixel_x=center_x,
                    pixel_y=center_y,
                    confidence=confidence,
                    class_name=mapped_class_name,
                    bbox=(int(x1), int(y1), int(x2), int(y2)),
                    target_type=TargetType.STATIC,  # Will be classified later by CoordinateTransformer
                    timestamp=current_time,
                    track_id=self.track_id_counter + i  # Simple ID assignment
                )
                
                detected_targets.append(detected_target)
                
            except Exception as e:
                if verbose:
                    print(f"⚠️ Error processing detection {i}: {e}")
                continue
        
        # Update counters
        self.detection_count += len(detected_targets)
        self.track_id_counter += len(detected_targets)
        
        return detected_targets
    
    def draw_detections(self, frame: np.ndarray, detections: List[DetectedTarget]) -> np.ndarray:
        """
        Draw detection results on frame
        
        Args:
            frame: Input image
            detections: List of DetectedTarget objects
            
        Returns:
            Annotated frame
        """
        annotated_frame = frame.copy()
        
        for detection in detections:
            x1, y1, x2, y2 = detection.bbox
            
            # Color based on class
            if detection.class_name == 'red_balloon':
                color = (0, 0, 255)  # Red
                label = "DÜŞMAN"
            elif detection.class_name == 'blue_balloon':
                color = (255, 0, 0)  # Blue
                label = "DOST"
            else:
                color = (0, 255, 0)  # Green
                label = detection.class_name.upper()
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            
            # Draw center point
            center = (int(detection.pixel_x), int(detection.pixel_y))
            cv2.circle(annotated_frame, center, 5, color, -1)
            
            # Draw label with confidence
            label_text = f"{label} {detection.confidence:.2f}"
            label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            
            # Background for text
            cv2.rectangle(annotated_frame, 
                         (x1, y1 - label_size[1] - 10), 
                         (x1 + label_size[0], y1), 
                         color, -1)
            
            # Text
            cv2.putText(annotated_frame, label_text, 
                       (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Draw crosshair at center
            cv2.line(annotated_frame, 
                    (center[0] - 10, center[1]), 
                    (center[0] + 10, center[1]), color, 2)
            cv2.line(annotated_frame, 
                    (center[0], center[1] - 10), 
                    (center[0], center[1] + 10), color, 2)
        
        # Draw detection count
        cv2.putText(annotated_frame, f"Hedefler: {len(detections)}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return annotated_frame
    
    def get_model_info(self) -> Dict[str, Any]:
        """Get model information"""
        if not self.is_loaded:
            return {"status": "not_loaded"}
        
        info = {
            "status": "loaded",
            "model_path": self.config.model_path,
            "total_detections": self.detection_count
        }
        
        if hasattr(self.model, 'names'):
            info["classes"] = list(self.model.names.values())
        
        return info


class MultiModelYOLOWrapper:
    """
    Wrapper for multiple YOLO models (like Mission1, Mission2)
    Compatible with existing Arayüz system
    """
    
    def __init__(self):
        """Initialize multi-model wrapper"""
        self.models = {}
        self.active_model_name = None
        self.active_detector = None
        
        print("Multi-model YOLO wrapper initialized")
    
    def add_model(self, name: str, model_path: str, config: YOLOConfig = None) -> bool:
        """Add a YOLO model to the wrapper"""
        if config is None:
            config = YOLOConfig(model_path=model_path)
        else:
            config.model_path = model_path
        
        try:
            detector = YOLODetectorWrapper(config)
            success = detector.load_model()
            
            if success:
                self.models[name] = detector
                print(f"✅ Added model '{name}': {model_path}")
                
                # Set as active if first model
                if self.active_model_name is None:
                    self.set_active_model(name)
                
                return True
            else:
                print(f"❌ Failed to add model '{name}': {model_path}")
                return False
                
        except Exception as e:
            print(f"❌ Error adding model '{name}': {e}")
            return False
    
    def set_active_model(self, name: str) -> bool:
        """Set active model"""
        if name in self.models:
            self.active_model_name = name
            self.active_detector = self.models[name]
            print(f"🎯 Active model set to: {name}")
            return True
        else:
            print(f"❌ Model '{name}' not found. Available: {list(self.models.keys())}")
            return False
    
    def detect(self, frame: np.ndarray, verbose: bool = False) -> List[DetectedTarget]:
        """Run detection with active model"""
        if self.active_detector is None:
            if verbose:
                print("⚠️ No active model set")
            return []
        
        return self.active_detector.detect(frame, verbose)
    
    def draw_detections(self, frame: np.ndarray, detections: List[DetectedTarget]) -> np.ndarray:
        """Draw detections using active model"""
        if self.active_detector is None:
            return frame
        
        return self.active_detector.draw_detections(frame, detections)
    
    def get_available_models(self) -> List[str]:
        """Get list of available models"""
        return list(self.models.keys())
    
    def get_active_model_info(self) -> Dict[str, Any]:
        """Get active model information"""
        if self.active_detector is None:
            return {"status": "no_active_model"}
        
        info = self.active_detector.get_model_info()
        info["model_name"] = self.active_model_name
        return info


# Factory function for easy setup
def create_yolo_wrapper_from_existing(mission1_path: str = None, mission2_path: str = None) -> MultiModelYOLOWrapper:
    """
    Create multi-model wrapper with existing Computer_Vision models
    
    Args:
        mission1_path: Path to Mission1 model (default: Computer_Vision/Mission1/Mission_1.pt)
        mission2_path: Path to Mission2 model (default: Computer_Vision/Mission2/v2_last.pt)
    """
    wrapper = MultiModelYOLOWrapper()
    
    # Default paths
    if mission1_path is None:
        mission1_path = "Computer_Vision/Mission1/Mission_1.pt"
    if mission2_path is None:
        mission2_path = "Computer_Vision/Mission2/v2_last.pt"
    
    # Add Mission1 model
    config1 = YOLOConfig(
        model_path=mission1_path,
        confidence_threshold=0.7,
        target_classes=['red_balloon', 'blue_balloon']  # Will map 'balloon' to 'red_balloon'
    )
    wrapper.add_model("Mission1", mission1_path, config1)
    
    # Add Mission2 model  
    config2 = YOLOConfig(
        model_path=mission2_path,
        confidence_threshold=0.7,
        target_classes=['red_balloon', 'blue_balloon']
    )
    wrapper.add_model("Mission2", mission2_path, config2)
    
    return wrapper


if __name__ == "__main__":
    # Test script
    print("🧪 Testing YOLO Wrapper...")
    
    # Create multi-model wrapper
    yolo_wrapper = create_yolo_wrapper_from_existing()
    
    print(f"Available models: {yolo_wrapper.get_available_models()}")
    print(f"Active model info: {yolo_wrapper.get_active_model_info()}")
    
    # Test with webcam (optional)
    try:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            print("📹 Testing with webcam... Press 'q' to quit, 'm' to switch models")
            
            models = yolo_wrapper.get_available_models()
            current_model_idx = 0
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                # Run detection
                detections = yolo_wrapper.detect(frame, verbose=True)
                
                # Draw results
                annotated_frame = yolo_wrapper.draw_detections(frame, detections)
                
                # Show active model
                active_info = yolo_wrapper.get_active_model_info()
                cv2.putText(annotated_frame, f"Model: {active_info.get('model_name', 'None')}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
                cv2.imshow('YOLO Wrapper Test', annotated_frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('m') and len(models) > 1:
                    # Switch models
                    current_model_idx = (current_model_idx + 1) % len(models)
                    yolo_wrapper.set_active_model(models[current_model_idx])
            
            cap.release()
            cv2.destroyAllWindows()
        else:
            print("📷 Webcam not available, skipping camera test")
            
    except KeyboardInterrupt:
        print("Test interrupted by user")
    except Exception as e:
        print(f"Test error: {e}")
    
    print("✅ YOLO Wrapper test complete!") 