# SafetyStatus.msg - Safety system status monitoring
# Header for timestamp
std_msgs/Header header

# Overall safety status
uint8 safety_level         # Current safety level
uint8 SAFETY_SECURE=0      # All safety systems operational
uint8 SAFETY_CAUTION=1     # Minor safety concerns
uint8 SAFETY_WARNING=2     # Major safety warnings
uint8 SAFETY_CRITICAL=3    # Critical safety violations
uint8 SAFETY_EMERGENCY=4   # Emergency stop required

# Physical safety systems
bool safety_switch_engaged  # Physical safety switch status
bool emergency_stop_clear   # Emergency stop button status
bool area_clear            # Area clearance sensors
bool operator_present      # Operator presence detection

# Pressure and mechanical systems
float32 system_pressure     # Current system pressure (PSI or bar)
float32 pressure_min        # Minimum safe pressure
float32 pressure_max        # Maximum safe pressure
bool pressure_sensor_ok     # Pressure sensor operational status
bool valve_operational      # Firing valve operational status

# Electrical safety
float32 voltage_level       # System voltage level
bool power_stable          # Power supply stability
bool gpio_systems_ok       # GPIO and control systems status
bool communication_ok      # Inter-system communication status

# Timing and operational safety
builtin_interfaces/Duration time_since_last_fire  # Time since last shot
builtin_interfaces/Duration safety_timeout        # Remaining safety timeout
bool firing_rate_ok        # Firing rate within safe limits
uint32 consecutive_shots   # Number of consecutive shots fired

# Environmental safety
float32 temperature        # System temperature (Celsius)
float32 humidity          # Environmental humidity (%)
bool temperature_ok       # Temperature within safe range
bool environmental_ok     # Overall environmental conditions

# Target area safety
bool target_area_clear    # Target area is clear of non-targets
bool friendly_targets_safe # No friendly targets in engagement zone
bool civilian_area_clear  # No civilian areas in firing line
float32 safe_firing_angle_min  # Minimum safe firing angle
float32 safe_firing_angle_max  # Maximum safe firing angle

# Safety violations and warnings
uint32 active_violations   # Number of active safety violations
uint32 warning_count      # Number of active warnings
string[] violations       # Array of safety violation descriptions
string[] warnings         # Array of safety warning descriptions

# System interlocks
bool motor_safety_ok      # Motor safety systems operational
bool weapon_safety_ok     # Weapon safety systems operational
bool vision_safety_ok     # Vision system safety checks passed
bool all_interlocks_ok    # All safety interlocks satisfied

# Override status (DANGEROUS!)
bool safety_override_active    # True if safety is being overridden
string override_reason        # Reason for safety override
string override_authorized_by # Who authorized the safety override
builtin_interfaces/Time override_expires  # When safety override expires 