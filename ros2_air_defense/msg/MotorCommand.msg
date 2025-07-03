# MotorCommand.msg - Motor movement commands
# Header for timestamp and command ID
std_msgs/Header header

# Command identification
uint32 command_id          # Unique command identifier
uint8 command_type         # Type of command
uint8 MOVE_TO_POSITION=0   # Move to specific position
uint8 MOVE_RELATIVE=1      # Move relative to current position
uint8 HOME_MOTORS=2        # Home both motors
uint8 STOP_MOTORS=3        # Stop all motor movement
uint8 EMERGENCY_STOP=4     # Emergency stop

# Target positions (in degrees)
float32 target_azimuth     # Target azimuth angle (-180 to +180)
float32 target_elevation   # Target elevation angle (-90 to +90)

# Movement parameters
float32 azimuth_speed      # Azimuth movement speed (0.0-1.0, 1.0=max speed)
float32 elevation_speed    # Elevation movement speed (0.0-1.0, 1.0=max speed)
float32 acceleration       # Acceleration factor (0.0-1.0)
bool synchronized          # True if both motors should move together

# Priority and timing
uint8 priority             # Command priority (0=low, 255=emergency)
builtin_interfaces/Duration timeout  # Command timeout
bool wait_for_completion   # True if sender wants completion confirmation

# Safety parameters
bool force_execution       # True to override safety checks (dangerous!)
bool check_limits          # True to check angle limits (recommended)
float32 max_deviation      # Maximum allowed deviation from target (degrees)

# Tracking information
uint32 target_id          # ID of target being tracked (0 if not tracking)
bool continuous_tracking  # True for continuous target tracking mode 