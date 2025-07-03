# ROS2 Air Defense System

🚀 **Advanced ROS2-based Air Defense System** for autonomous target engagement using computer vision, precision motor control, and pneumatic firing system.

## 🏗️ System Architecture

```
📡 Vision Bridge Node     ──→  🎯 Coordinate Transformer  ──→  🤖 Orchestrator
       │                           │                            │
       ↓                           ↓                            ↓
   YOLO Detection              Motor Angles               Strategy Control
       │                           │                            │
       └──→ Camera Feed        Motor Controller  ←──────────────┘
                                   │
                              Firing Controller
```

### 🔧 Node Architecture

| Node | Responsibility | Key Features |
|------|----------------|--------------|
| **Vision Bridge** | YOLO integration | Real-time detection, ROS2 message conversion |
| **Coordinate Transformer** | Pixel → Motor angles | Target tracking, priority calculation |
| **Motor Controller** | Stepper motor control | Dual motor system, safety limits |
| **Firing Controller** | Weapon system | Pneumatic valve, comprehensive safety |
| **Orchestrator** | Main coordination | Scan-Plan-Execute strategy |

## 🚀 Quick Start

### 1. Install Dependencies

```bash
# ROS2 Humble (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop-full

# Python dependencies
pip install ultralytics opencv-python numpy

# Hardware control (Jetson Nano)
pip install Jetson.GPIO  # or RPi.GPIO for Raspberry Pi
```

### 2. Build the Package

```bash
cd ~/ros2_ws/src
git clone <your-repo>
cd ~/ros2_ws
colcon build --packages-select ros2_air_defense
source install/setup.bash
```

### 3. Launch the System

```bash
# Simulation Mode (safe for testing)
ros2 launch ros2_air_defense air_defense_system.launch.py mode:=simulation

# Hardware Mode (with real GPIO control)
ros2 launch ros2_air_defense air_defense_system.launch.py \
    mode:=hardware enable_gpio:=true

# Autonomous Mode (advanced users only)
ros2 launch ros2_air_defense air_defense_system.launch.py \
    targeting_mode:=autonomous auto_startup:=true
```

## 🎮 Operation Modes

### 🔧 Manual Mode
- **Human operator control**
- Manual target selection and engagement
- Real-time camera feed with overlay
- Safety confirmations required

### 🤖 Semi-Autonomous Mode  
- **Computer-assisted targeting**
- Automatic target detection and prioritization
- Operator confirmation required for firing
- Enhanced situational awareness

### 🎯 Autonomous Mode
- **Fully autonomous operation**
- Scan-Plan-Execute strategy
- Automatic enemy target engagement
- Comprehensive safety monitoring

## 📊 ROS2 Topics & Services

### 📡 Key Topics

```bash
# Vision System
/air_defense/detected_targets        # Raw YOLO detections
/air_defense/processed_targets       # Processed with motor angles
/air_defense/camera/raw_image        # Live camera feed
/air_defense/camera/annotated_image  # Detection overlay

# Motor System  
/air_defense/motor_position          # Current motor position
/air_defense/motor_commands          # Movement commands

# Firing System
/air_defense/safety_status           # Safety system status
/air_defense/firing_commands         # Weapon control

# System Status
/air_defense/system_status           # Overall system health
```

### 🛠️ Services & Actions

```bash
# System Control
ros2 service call /air_defense/arm_system ros2_air_defense/srv/ArmSystem
ros2 service call /air_defense/emergency_stop ros2_air_defense/srv/EmergencyStop

# Motor Control
ros2 action send_goal /air_defense/move_to_position ros2_air_defense/action/MoveToPosition

# Target Engagement
ros2 action send_goal /air_defense/engage_target ros2_air_defense/action/EngageTarget
```

## ⚙️ Configuration

### 📁 Configuration Files

- `config/air_defense_config.yaml` - Main system configuration
- `config/camera_config.json` - Camera calibration
- `config/motor_config.json` - Motor parameters  
- `config/firing_config.json` - Weapon system settings

### 🎛️ Key Parameters

```yaml
# Target Detection
vision:
  confidence_threshold: 0.7
  target_classes: ["red_balloon", "blue_balloon"]

# Motor Limits
motors:
  limits:
    azimuth_max: 180.0    # degrees
    elevation_max: 90.0   # degrees

# Safety Settings
safety:
  min_fire_interval: 1.0  # seconds
  max_consecutive_shots: 5
  require_authorization: true
```

## 🛡️ Safety Systems

### 🚨 Multi-Layer Safety

1. **Physical Safety Switch** - Hardware emergency stop
2. **Software Interlocks** - Multiple safety checks
3. **Firing Rate Limits** - Prevents system abuse
4. **Target Confirmation** - Enemy/friendly verification
5. **Area Clearance** - Ensures safe firing zones

### ⚠️ Safety Features

- **Emergency Stop** - Immediate system shutdown
- **Auto-Disarm** - Automatic weapon disarming
- **Pressure Monitoring** - Pneumatic system safety
- **Thermal Protection** - Prevents overheating
- **Authorization Required** - Human oversight

## 🎯 Teknofest Competition Ready

### 🏆 Competition Features

- **30-second startup time** ⏱️
- **±2° targeting accuracy** 🎯
- **Red=Enemy, Blue=Friendly** 🔴🔵
- **Autonomous operation** 🤖
- **Safety compliance** 🛡️

### 📈 Performance Metrics

- **Detection Rate**: 30 FPS
- **Motor Response**: <200ms
- **Engagement Time**: 2-3 seconds
- **Accuracy**: ±2° targeting precision

## 🔧 Development & Testing

### 🧪 Testing Commands

```bash
# Test individual nodes
ros2 run ros2_air_defense vision_bridge_node
ros2 run ros2_air_defense motor_controller_node

# Monitor system status
ros2 topic echo /air_defense/system_status

# Emergency stop (testing)
ros2 service call /air_defense/emergency_stop \
  ros2_air_defense/srv/EmergencyStop "{reason: 'test'}"
```

### 📊 System Monitoring

```bash
# View system graph
rqt_graph

# Monitor topics
ros2 topic list
ros2 topic hz /air_defense/detected_targets

# Check node status
ros2 node list
ros2 node info /air_defense_orchestrator_node
```

## 🔌 Hardware Integration

### 📡 Supported Platforms

- **Jetson Nano JNX30D** (primary)
- **Raspberry Pi 4** (alternative)
- **Ubuntu 22.04** (simulation)

### 🔧 GPIO Mapping

```
Azimuth Motor:  Pins 18-20
Elevation Motor: Pins 21-23  
Firing Valve:   Pin 16
Safety Switch:  Pin 17
Emergency Stop: Pin 27
```

### 📷 Camera Support

- USB cameras (UVC compatible)
- CSI cameras (Jetson/Pi)
- IP cameras (with modification)

## 🐛 Troubleshooting

### ❌ Common Issues

**YOLO Model Not Found**
```bash
# Ensure model files are in correct location
ls Computer_Vision/Mission1/Mission_1.pt
```

**GPIO Permission Denied**
```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER
# Logout and login again
```

**ROS2 Node Communication Issues**
```bash
# Check ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42
# Restart nodes
```

### 📞 Support

- 📧 Email: support@airdefense.com
- 🐛 Issues: GitHub Issues
- 📖 Docs: `/docs` folder

## 📜 License & Credits

- **License**: MIT License
- **YOLO Integration**: Ultralytics YOLOv8
- **ROS2**: Open Source Robotics Foundation
- **Hardware Control**: GPIO libraries

## 🚀 Future Enhancements

- [ ] **Multi-target engagement**
- [ ] **Advanced tracking algorithms**  
- [ ] **Machine learning improvements**
- [ ] **Web-based monitoring dashboard**
- [ ] **Real-time telemetry**

---

## ⚠️ SAFETY DISCLAIMER

**This system includes weapon control capabilities. Always:**
- Follow local laws and regulations
- Implement proper safety measures
- Never operate without human oversight
- Use only in designated testing areas
- Maintain emergency stop capability

**🔴 LIVE WEAPON SYSTEM - EXTREME CAUTION REQUIRED 🔴**

---

*Built for Teknofest 2024 🇹🇷 - Autonomous Air Defense Competition* 