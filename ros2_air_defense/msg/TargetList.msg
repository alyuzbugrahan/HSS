# TargetList.msg - List of detected targets
# Header for timestamp
std_msgs/Header header

# Array of detected targets
DetectedTarget[] targets

# Summary information
uint32 total_targets       # Total number of targets detected
uint32 enemy_targets       # Number of enemy targets (red balloons)
uint32 friendly_targets    # Number of friendly targets (blue balloons)
uint32 moving_targets      # Number of moving targets
uint32 static_targets      # Number of static targets

# Best target information
bool has_priority_target   # True if there's a priority target
uint32 priority_target_id  # ID of highest priority target
float32 best_target_azimuth    # Azimuth of best target
float32 best_target_elevation  # Elevation of best target

# Detection statistics
float32 detection_confidence   # Average confidence of all detections
uint32 detection_count        # Running count of total detections
builtin_interfaces/Duration processing_time  # Time taken to process this frame 