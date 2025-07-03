# MotorPosition.msg - Current motor positions
# Header for timestamp
std_msgs/Header header

# Current motor positions in degrees
float32 azimuth_angle      # Base motor azimuth position (-180 to +180 degrees)
float32 elevation_angle    # Elevation motor position (-90 to +90 degrees)

# Motor status information
bool azimuth_homed         # True if azimuth motor is homed
bool elevation_homed       # True if elevation motor is homed
bool azimuth_moving        # True if azimuth motor is currently moving
bool elevation_moving      # True if elevation motor is currently moving

# Motor limits and capabilities
float32 azimuth_min_angle  # Minimum azimuth angle
float32 azimuth_max_angle  # Maximum azimuth angle  
float32 elevation_min_angle # Minimum elevation angle
float32 elevation_max_angle # Maximum elevation angle

# Motor step information
uint32 azimuth_steps       # Current step position of azimuth motor
uint32 elevation_steps     # Current step position of elevation motor
float32 azimuth_speed      # Current azimuth motor speed (steps/sec)
float32 elevation_speed    # Current elevation motor speed (steps/sec)

# System status
bool system_ready          # True if motor system is ready for commands
bool emergency_stop        # True if emergency stop is active
string status_message      # Human readable status message 