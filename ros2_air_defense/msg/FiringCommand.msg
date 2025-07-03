# FiringCommand.msg - Weapon firing commands
# Header for timestamp and command tracking
std_msgs/Header header

# Command identification
uint32 command_id          # Unique command identifier
uint8 command_type         # Type of firing command
uint8 FIRE_SINGLE=0        # Fire single shot
uint8 FIRE_BURST=1         # Fire burst of shots
uint8 ARM_WEAPON=2         # Arm weapon system
uint8 DISARM_WEAPON=3      # Disarm weapon system
uint8 TEST_FIRING=4        # Test firing system (no projectile)
uint8 EMERGENCY_CEASE=5    # Emergency cease fire

# Target information
uint32 target_id           # ID of target being engaged
float32 target_azimuth     # Target azimuth position
float32 target_elevation   # Target elevation position
float32 target_priority    # Target priority score
string target_class        # Target class ("red_balloon", "blue_balloon")

# Firing parameters
uint8 shot_count           # Number of shots to fire (for burst mode)
float32 firing_delay       # Delay between shots in burst (seconds)
float32 valve_duration     # Duration to keep valve open (seconds)
float32 pressure_setting   # Pressure setting (0.0-1.0)

# Safety parameters
bool override_safety       # True to override safety checks (DANGEROUS!)
bool confirm_target        # True if target identification is confirmed
bool manual_authorization  # True if manual authorization is given
string authorization_code  # Authorization code for firing

# Engagement strategy
bool lead_target           # True to lead moving targets
float32 lead_time          # Time to lead target by (seconds)
bool continuous_tracking   # True for continuous target tracking during firing
uint8 engagement_priority  # Engagement priority (0=low, 255=critical)

# Result expectations
bool wait_for_confirmation # True if sender wants firing confirmation
builtin_interfaces/Duration timeout  # Command timeout
uint8 retry_count          # Number of retries if firing fails

# Context information
string mission_id          # Mission or operation ID
string operator_id         # Operator who authorized the command
builtin_interfaces/Time authorized_at  # When command was authorized 