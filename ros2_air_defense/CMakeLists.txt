cmake_minimum_required(VERSION 3.8)
project(ros2_air_defense)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Message files
  "msg/DetectedTarget.msg"
  "msg/TargetList.msg"
  "msg/MotorPosition.msg"
  "msg/MotorCommand.msg"
  "msg/SystemStatus.msg"
  "msg/FiringCommand.msg"
  "msg/SafetyStatus.msg"
  
  # Service files
  "srv/ArmSystem.srv"
  "srv/DisarmSystem.srv"
  "srv/EmergencyStop.srv"
  "srv/HomeMotors.srv"
  "srv/SetTargetingMode.srv"
  
  # Action files
  "action/EngageTarget.action"
  "action/MoveToPosition.action"
  "action/ScanArea.action"
  
  DEPENDENCIES 
    builtin_interfaces
    std_msgs
    geometry_msgs
    sensor_msgs
    action_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/vision_bridge_node.py
  scripts/coordinate_transformer_node.py
  scripts/motor_controller_node.py
  scripts/firing_controller_node.py
  scripts/target_tracker_node.py
  scripts/air_defense_orchestrator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package() 