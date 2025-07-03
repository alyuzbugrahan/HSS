# 🚀 Jetson Nano Hızlı Deployment Rehberi

## 🎯 Tek Komutla Kurulum

```bash
# GitHub'dan clone et
cd ~
git clone https://github.com/yourusername/ROS_test.git
cd ROS_test

# Otomatik kurulum
chmod +x scripts/jetson_setup.sh
./scripts/jetson_setup.sh

# Sistem restart
sudo reboot
```

## ✅ Test Etme

```bash
# Kurulum sonrası test
cd ~/ROS_test
chmod +x scripts/test_system.sh
./scripts/test_system.sh
```

## 🚀 Sistem Başlatma

### Simulation Mode (Hardware Olmadan)
```bash
cd ~/ROS_test
source install/setup.bash
ros2 launch ros2_air_defense air_defense_system.launch.py simulation_mode:=true
```

### Hardware Mode (Tam Sistem)
```bash
cd ~/ROS_test
source install/setup.bash
ros2 launch ros2_air_defense air_defense_system.launch.py enable_gpio:=true
```

## 🔧 İzleme ve Kontrol

```bash
# Sistem durumu
ros2 topic echo /air_defense/air_defense_status

# Manuel motor kontrolü
ros2 service call /air_defense/home_motors ros2_air_defense/srv/HomeMotors

# Emergency stop
ros2 service call /air_defense/emergency_stop_firing ros2_air_defense/srv/EmergencyStop
```

## 📊 ROS2 Araçları

```bash
# Node'ları görüntüle
ros2 node list

# Topic'leri görüntüle
ros2 topic list

# Grafik görüntüleme
ros2 run rqt_graph rqt_graph
```

## 🐛 Hata Giderme

### GPIO Permission Hatası
```bash
sudo usermod -a -G gpio $USER
sudo reboot
```

### Camera Bulunamadı
```bash
ls /dev/video*
# Camera ID'yi config'te ayarla
```

### YOLO Model Hatası
```bash
# Model dosyalarının varlığını kontrol et
ls Computer_Vision/Mission*/
```

## 📞 Destek

Detaylı kurulum rehberi: `JETSON_SETUP.md`
Test scripti: `scripts/test_system.sh`
Sistem durumu: `./scripts/test_system.sh`

## 🎯 Hızlı Komutlar

```bash
# Kurulum
./scripts/jetson_setup.sh

# Test
./scripts/test_system.sh

# Başlat (Simülasyon)
ros2 launch ros2_air_defense air_defense_system.launch.py simulation_mode:=true

# Başlat (Hardware)
ros2 launch ros2_air_defense air_defense_system.launch.py enable_gpio:=true
``` 