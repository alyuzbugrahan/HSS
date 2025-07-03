# 🤖 Jetson Nano Kurulum Rehberi - Hava Savunma Sistemi

## 🎯 Ön Gereksinimler
- Jetson Nano Developer Kit
- MicroSD Kart (64GB+ önerilen)
- USB Webcam veya CSI Camera
- Stepper motor driver'lar ve motorlar
- Pnömatik valf sistemi
- GPIO breadboard ve bağlantı kabloları

## 📦 Sistem Kurulumu

### 1️⃣ Jetson Nano OS Kurulumu
```bash
# JetPack 4.6.1 veya daha yeni sürüm gerekli
# NVIDIA'nın resmi imajını SD karta yaz
# İlk boot sonrası:

sudo apt update && sudo apt upgrade -y
sudo apt install curl wget git nano vim -y
```

### 2️⃣ ROS2 Humble Kurulumu
```bash
# ROS2 GPG anahtarını ekle
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# ROS2 repository ekle
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# ROS2 Humble kurulumu
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-ros-base -y

# Development tools
sudo apt install python3-pip python3-colcon-common-extensions -y
sudo apt install ros-humble-cv-bridge ros-humble-image-transport -y
```

### 3️⃣ Python Dependencies
```bash
# Virtual environment oluştur
python3 -m pip install --user virtualenv
python3 -m venv ~/air_defense_env
source ~/air_defense_env/bin/activate

# Required packages
pip install ultralytics opencv-python numpy
pip install Jetson.GPIO adafruit-circuitpython-gpio
pip install PyQt5 # GUI için (opsiyonel)

# ROS2 Python dependencies
pip install rclpy sensor-msgs-py
```

### 4️⃣ GPIO ve Hardware Setup
```bash
# GPIO group'a kullanıcı ekle
sudo usermod -a -G gpio $USER
sudo usermod -a -G i2c $USER
sudo usermod -a -G spi $USER

# GPIO permissions
sudo chmod 666 /dev/gpiomem

# Reboot gerekebilir
sudo reboot
```

## 🔧 Proje Kurulumu

### 1️⃣ GitHub'dan Clone
```bash
cd ~
git clone https://github.com/yourusername/ROS_test.git
cd ROS_test
```

### 2️⃣ ROS2 Environment Setup
```bash
# ROS2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/air_defense_env/bin/activate" >> ~/.bashrc
source ~/.bashrc

# Workspace build
cd ~/ROS_test
colcon build --packages-select ros2_air_defense
source install/setup.bash
```

### 3️⃣ Configuration Dosyalarını Ayarla
```bash
# Motor config düzenle
nano config/motor_config.json
# GPIO pinlerini kontrol et

# Camera config düzenle  
nano config/camera_config.json
# Camera ID ve FOV ayarla

# Firing config düzenle
nano config/firing_config.json
# Safety ayarlarını kontrol et
```

## 🧪 Hardware Test Prosedürleri

### 1️⃣ GPIO Test
```bash
# ROS2 environment source et
source ~/ROS_test/install/setup.bash

# GPIO test node
python3 -c "
import Jetson.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
for i in range(5):
    GPIO.output(18, GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(18, GPIO.LOW)
    time.sleep(0.5)
GPIO.cleanup()
print('GPIO test tamamlandı!')
"
```

### 2️⃣ Camera Test
```bash
# Camera bağlantısını test et
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if ret:
    print(f'Camera OK: {frame.shape}')
    cv2.imwrite('test_camera.jpg', frame)
else:
    print('Camera ERROR!')
cap.release()
"
```

### 3️⃣ YOLO Model Test
```bash
# YOLO model yükleme testi
cd ~/ROS_test
python3 -c "
import sys
sys.path.append('src')
from src.vision.yolo_detector import create_yolo_wrapper_from_existing
wrapper = create_yolo_wrapper_from_existing()
print(f'YOLO models: {wrapper.get_available_models()}')
"
```

## 🚀 Sistem Başlatma

### 1️⃣ Full System Launch
```bash
# Terminal 1: Ana sistem
cd ~/ROS_test
source install/setup.bash
ros2 launch ros2_air_defense air_defense_system.launch.py

# Terminal 2: Sistem durumu izleme
ros2 topic echo /air_defense/air_defense_status

# Terminal 3: Manuel komutlar
ros2 service call /air_defense/home_motors ros2_air_defense/srv/HomeMotors
```

### 2️⃣ Simulation Mode (Hardware olmadan test)
```bash
ros2 launch ros2_air_defense air_defense_system.launch.py \
  enable_gpio:=false \
  enable_safety:=false \
  simulation_mode:=true
```

### 3️⃣ Individual Node Testing
```bash
# Sadece vision test
ros2 run ros2_air_defense vision_bridge_node.py

# Sadece motor test  
ros2 run ros2_air_defense motor_controller_node.py

# Sadece firing test
ros2 run ros2_air_defense firing_controller_node.py
```

## 🔧 Debug ve Troubleshooting

### 1️⃣ ROS2 Node Status
```bash
ros2 node list
ros2 topic list  
ros2 service list
ros2 param list
```

### 2️⃣ Log İnceleme
```bash
# Node logları
ros2 log set-level ros2_air_defense.vision_bridge_node DEBUG

# Sistem logları
tail -f ~/.ros/log/latest/ros2_air_defense*/*.log
```

### 3️⃣ Performance Monitoring
```bash
# CPU kullanımı
htop

# GPU kullanımı  
sudo tegrastats

# ROS2 performance
ros2 run rqt_graph rqt_graph
```

## ⚡ Performance Optimizations

### 1️⃣ Jetson Power Mode
```bash
# Max performance mode
sudo nvpmodel -m 0
sudo jetson_clocks
```

### 2️⃣ YOLO Optimization
```bash
# TensorRT optimization (opsiyonel)
pip install torch2trt
# Model'i TensorRT'ye convert et
```

### 3️⃣ Memory Management
```bash
# Swap file oluştur
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## 🛡️ Güvenlik Kontrolleri

### 1️⃣ Hardware Safety
```bash
# GPIO pin durumlarını kontrol et
cat /sys/kernel/debug/gpio

# Safety switch test
python3 -c "
import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
print(f'Safety switch: {GPIO.input(18)}')
GPIO.cleanup()
"
```

### 2️⃣ Emergency Stop Test
```bash
# Emergency stop servisi test
ros2 service call /air_defense/emergency_stop_firing ros2_air_defense/srv/EmergencyStop
```

## 🎯 Production Checklist

- [ ] ROS2 kurulumu tamamlandı
- [ ] GPIO permissions ayarlandı  
- [ ] Camera bağlantısı test edildi
- [ ] YOLO modelleri yüklendi
- [ ] Motor connections test edildi
- [ ] Safety systems test edildi
- [ ] Emergency stop çalışıyor
- [ ] Full system launch başarılı
- [ ] Performance acceptable
- [ ] All configurations verified

## 📞 Hata Durumunda

### Common Issues:
1. **GPIO Permission Error**: `sudo usermod -a -G gpio $USER` ve reboot
2. **Camera Not Found**: USB camera ID kontrol et (`ls /dev/video*`)
3. **YOLO Model Error**: Model dosya yollarını kontrol et
4. **ROS2 Import Error**: Environment setup kontrol et
5. **Memory Issues**: Swap file oluştur

### Support Commands:
```bash
# System info
uname -a
lsb_release -a
nvidia-smi

# ROS2 diagnostics
ros2 doctor
ros2 wtf
``` 