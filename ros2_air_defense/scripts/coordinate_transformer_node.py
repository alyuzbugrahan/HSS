#!/usr/bin/env python3
"""
Coordinate Transformer Node for ROS2 Air Defense System
"""

import rclpy
from rclpy.node import Node
from ros2_air_defense.msg import DetectedTarget, TargetList

class CoordinateTransformerNode(Node):
    def __init__(self):
        super().__init__('coordinate_transformer_node')
        
        # Subscribers
        self.target_sub = self.create_subscription(
            TargetList,
            'detected_targets',
            self.target_callback,
            10
        )
        
        # Publishers
        self.processed_pub = self.create_publisher(
            TargetList,
            'processed_targets',
            10
        )
        
        self.get_logger().info("Coordinate Transformer Node started")
    
    def target_callback(self, msg):
        """Process detected targets"""
        processed_msg = TargetList()
        processed_msg.header = msg.header
        
        for target in msg.targets:
            # Convert pixel to motor angles
            processed_target = self.convert_target(target)
            processed_msg.targets.append(processed_target)
        
        self.processed_pub.publish(processed_msg)
    
    def convert_target(self, target):
        """Convert pixel coordinates to motor angles"""
        # Simple conversion - will be enhanced
        processed = target  # For now, pass through
        return processed

def main():
    rclpy.init()
    node = CoordinateTransformerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 