# DetectedTarget.msg - Detected target information
# Header for timestamp and frame information
std_msgs/Header header

# Target identification
uint32 target_id
string class_name          # "red_balloon", "blue_balloon", etc.
float32 confidence         # YOLO confidence score (0.0-1.0)

# Pixel coordinates (from camera)
float32 pixel_x
float32 pixel_y
uint32 bbox_x1            # Bounding box coordinates
uint32 bbox_y1
uint32 bbox_x2  
uint32 bbox_y2

# Motor angles (converted coordinates)
float32 azimuth           # Motor azimuth angle in degrees
float32 elevation         # Motor elevation angle in degrees

# Target classification
uint8 target_type         # 0=STATIC, 1=MOVING, 2=UNKNOWN
uint8 TARGET_STATIC=0
uint8 TARGET_MOVING=1
uint8 TARGET_UNKNOWN=2

# Target priority and distance
float32 priority          # Calculated priority score
float32 distance_to_center # Distance from image center

# Tracking information
uint32 track_id           # Tracking ID for this target
float32 velocity_x        # Estimated velocity in pixels/sec
float32 velocity_y        # Estimated velocity in pixels/sec
bool is_enemy             # True if target is enemy (red balloon)

# Engagement status
bool is_engaged           # True if target is being engaged
bool is_locked            # True if target is locked for engagement 