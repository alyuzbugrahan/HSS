#!/usr/bin/env python3
"""
Main launch file for ROS2 Air Defense System
Launches all nodes with appropriate configurations
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParametersFromFile
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for complete air defense system"""
    
    # Package directory
    pkg_dir = get_package_share_directory('ros2_air_defense')
    config_dir = os.path.join(pkg_dir, 'config')
    
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'mode',
            default_value='simulation',
            choices=['simulation', 'hardware', 'test'],
            description='System mode: simulation, hardware, or test'
        ),
        DeclareLaunchArgument(
            'targeting_mode',
            default_value='manual',
            choices=['manual', 'autonomous', 'semi_autonomous'],
            description='Targeting mode for the system'
        ),
        DeclareLaunchArgument(
            'enable_gpio',
            default_value='false',
            description='Enable GPIO hardware control'
        ),
        DeclareLaunchArgument(
            'camera_id',
            default_value='0',
            description='Camera device ID'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='air_defense_config.yaml',
            description='Configuration file name'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            choices=['debug', 'info', 'warn', 'error'],
            description='Logging level for all nodes'
        ),
        DeclareLaunchArgument(
            'enable_safety_systems',
            default_value='true',
            description='Enable comprehensive safety systems'
        ),
        DeclareLaunchArgument(
            'auto_startup',
            default_value='false',
            description='Automatically start system initialization'
        ),
    ]
    
    # Get launch configurations
    mode = LaunchConfiguration('mode')
    targeting_mode = LaunchConfiguration('targeting_mode')
    enable_gpio = LaunchConfiguration('enable_gpio')
    camera_id = LaunchConfiguration('camera_id')
    config_file = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')
    enable_safety = LaunchConfiguration('enable_safety_systems')
    auto_startup = LaunchConfiguration('auto_startup')
    
    # Common parameters for all nodes
    common_parameters = [
        {'use_sim_time': False},
        {'log_level': log_level},
    ]
    
    # Vision Bridge Node
    vision_bridge_node = Node(
        package='ros2_air_defense',
        executable='vision_bridge_node.py',
        name='vision_bridge_node',
        output='screen',
        parameters=common_parameters + [
            {'camera_id': camera_id},
            {'publish_rate': 30.0},
            {'confidence_threshold': 0.7},
            {'publish_annotated_image': True},
            {'target_classes': ['red_balloon', 'blue_balloon']},
        ],
        remappings=[
            ('detected_targets', '/air_defense/detected_targets'),
            ('camera/raw_image', '/air_defense/camera/raw_image'),
            ('camera/annotated_image', '/air_defense/camera/annotated_image'),
        ]
    )
    
    # Coordinate Transformer Node
    coordinate_transformer_node = Node(
        package='ros2_air_defense',
        executable='coordinate_transformer_node.py',
        name='coordinate_transformer_node',
        output='screen',
        parameters=common_parameters + [
            {'camera_config_path': PathJoinSubstitution([config_dir, 'camera_config.json'])},
            {'movement_threshold': 5.0},
            {'target_timeout': 2.0},
            {'priority_update_rate': 10.0},
            {'enable_target_tracking': True},
        ],
        remappings=[
            ('detected_targets', '/air_defense/detected_targets'),
            ('processed_targets', '/air_defense/processed_targets'),
            ('priority_target', '/air_defense/priority_target'),
            ('motor_position', '/air_defense/motor_position'),
            ('motor_commands', '/air_defense/motor_commands'),
        ]
    )
    
    # Motor Controller Node
    motor_controller_node = Node(
        package='ros2_air_defense',
        executable='motor_controller_node.py',
        name='motor_controller_node',
        output='screen',
        parameters=common_parameters + [
            {'motor_config_path': PathJoinSubstitution([config_dir, 'motor_config.json'])},
            {'position_update_rate': 50.0},
            {'enable_gpio': enable_gpio},
            {'max_azimuth': 180.0},
            {'max_elevation': 90.0},
        ],
        remappings=[
            ('motor_commands', '/air_defense/motor_commands'),
            ('motor_position', '/air_defense/motor_position'),
            ('motor_system_status', '/air_defense/motor_system_status'),
            ('move_to_position', '/air_defense/move_to_position'),
            ('home_motors', '/air_defense/home_motors'),
            ('emergency_stop', '/air_defense/emergency_stop_motors'),
        ]
    )
    
    # Firing Controller Node
    firing_controller_node = Node(
        package='ros2_air_defense',
        executable='firing_controller_node.py',
        name='firing_controller_node',
        output='screen',
        parameters=common_parameters + [
            {'firing_config_path': PathJoinSubstitution([config_dir, 'firing_config.json'])},
            {'safety_update_rate': 10.0},
            {'enable_gpio': enable_gpio},
            {'require_authorization': enable_safety},
            {'max_consecutive_shots': 5},
            {'min_fire_interval': 1.0},
        ],
        remappings=[
            ('firing_commands', '/air_defense/firing_commands'),
            ('safety_status', '/air_defense/safety_status'),
            ('firing_system_status', '/air_defense/firing_system_status'),
            ('arm_firing_system', '/air_defense/arm_system'),
            ('disarm_firing_system', '/air_defense/disarm_system'),
            ('emergency_stop_firing', '/air_defense/emergency_stop_firing'),
        ],
        condition=IfCondition(enable_safety)
    )
    
    # Air Defense Orchestrator Node
    orchestrator_node = Node(
        package='ros2_air_defense',
        executable='air_defense_orchestrator_node.py',
        name='air_defense_orchestrator_node',
        output='screen',
        parameters=common_parameters + [
            {'targeting_mode': targeting_mode},
            {'auto_engagement_enabled': False},  # Safety: disabled by default
            {'scan_on_startup': auto_startup},
            {'engagement_timeout': 30.0},
            {'priority_threshold': 0.7},
            {'max_engagement_distance': 150.0},
        ],
        remappings=[
            ('processed_targets', '/air_defense/processed_targets'),
            ('motor_position', '/air_defense/motor_position'),
            ('safety_status', '/air_defense/safety_status'),
            ('motor_commands', '/air_defense/motor_commands'),
            ('firing_commands', '/air_defense/firing_commands'),
            ('air_defense_status', '/air_defense/system_status'),
            ('engage_target', '/air_defense/engage_target'),
            ('move_to_position', '/air_defense/move_to_position'),
            ('scan_area', '/air_defense/scan_area'),
            ('set_targeting_mode', '/air_defense/set_targeting_mode'),
        ]
    )
    
    # Hardware-specific nodes group
    hardware_nodes = GroupAction([
        # Additional hardware monitoring nodes could go here
    ], condition=IfCondition(enable_gpio))
    
    # Simulation-specific nodes group  
    simulation_nodes = GroupAction([
        # Simulation-specific nodes could go here
    ], condition=UnlessCondition(enable_gpio))
    
    # Safety monitoring node (always enabled)
    safety_monitor_node = Node(
        package='ros2_air_defense',
        executable='safety_monitor_node.py',
        name='safety_monitor_node',
        output='screen',
        parameters=common_parameters + [
            {'monitoring_rate': 20.0},
            {'emergency_stop_enabled': True},
            {'safety_timeout': 5.0},
        ],
        remappings=[
            ('safety_status', '/air_defense/safety_status'),
            ('system_status', '/air_defense/system_status'),
            ('emergency_stop', '/air_defense/emergency_stop_all'),
        ],
        condition=IfCondition(enable_safety)
    )
    
    return LaunchDescription(
        launch_args + [
            # Core system nodes
            vision_bridge_node,
            coordinate_transformer_node,
            motor_controller_node,
            firing_controller_node,
            orchestrator_node,
            
            # Conditional node groups
            hardware_nodes,
            simulation_nodes,
            safety_monitor_node,
        ]
    ) 