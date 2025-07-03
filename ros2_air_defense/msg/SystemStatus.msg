# SystemStatus.msg - Overall air defense system status
# Header for timestamp
std_msgs/Header header

# System identification and version
string system_name         # "Air Defense System"
string version             # System version
uint32 uptime_seconds      # System uptime in seconds

# Overall system state
uint8 system_state         # Current system state
uint8 STATE_OFFLINE=0      # System offline
uint8 STATE_INITIALIZING=1 # System starting up
uint8 STATE_STANDBY=2      # System ready, disarmed
uint8 STATE_ARMED=3        # System armed and ready
uint8 STATE_ENGAGING=4     # Actively engaging targets
uint8 STATE_EMERGENCY=5    # Emergency stop active
uint8 STATE_ERROR=6        # System error

# Subsystem status
bool vision_system_online     # YOLO detection system status
bool motor_system_online      # Motor control system status
bool firing_system_online     # Weapon system status
bool coordinate_system_online # Coordinate transformation status

# Operational statistics
uint32 total_targets_detected  # Total targets detected since startup
uint32 total_shots_fired      # Total shots fired since startup
uint32 successful_engagements # Number of successful target engagements
uint32 failed_engagements     # Number of failed target engagements

# Current operation
uint32 active_targets         # Number of currently tracked targets
uint32 engaged_targets        # Number of targets being engaged
string current_mode           # "manual", "autonomous", "test"
string active_mission         # Current mission or operation

# System health
float32 cpu_usage            # CPU usage percentage (0.0-100.0)
float32 memory_usage         # Memory usage percentage (0.0-100.0)
float32 temperature          # System temperature (Celsius)
bool safety_systems_ok       # True if all safety systems are operational

# Error and warning information
uint32 error_count          # Number of active errors
uint32 warning_count        # Number of active warnings
string[] error_messages     # Array of current error messages
string[] warning_messages   # Array of current warning messages

# Network and communication
bool ros2_communication_ok  # True if ROS2 communication is healthy
uint32 message_rate         # Messages per second being processed
builtin_interfaces/Duration last_heartbeat  # Time since last heartbeat 