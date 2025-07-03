#!/bin/bash
# 🤖 Jetson Nano Otomatik Kurulum Scripti
# Hava Savunma Sistemi için

set -e  # Exit on any error

echo "🎯 Jetson Nano Hava Savunma Sistemi Kurulumu Başlatılıyor..."

# Renk kodları
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Fonksiyonlar
print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

# Sistem güncelleme
print_info "Sistem paketleri güncelleniyor..."
sudo apt update && sudo apt upgrade -y
sudo apt install curl wget git nano vim htop -y
print_success "Sistem güncelleme tamamlandı"

# ROS2 kurulumu kontrolü
if ! command -v ros2 &> /dev/null; then
    print_info "ROS2 Humble kuruluyor..."
    
    # ROS2 GPG anahtarı
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    
    # ROS2 repository
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    
    # ROS2 packages
    sudo apt update
    sudo apt install ros-humble-desktop -y
    sudo apt install ros-humble-ros-base -y
    sudo apt install python3-pip python3-colcon-common-extensions -y
    sudo apt install ros-humble-cv-bridge ros-humble-image-transport -y
    
    print_success "ROS2 Humble kurulumu tamamlandı"
else
    print_success "ROS2 zaten kurulu"
fi

# Python environment
print_info "Python ortamı hazırlanıyor..."
python3 -m pip install --user virtualenv

if [ ! -d "$HOME/air_defense_env" ]; then
    python3 -m venv ~/air_defense_env
    print_success "Virtual environment oluşturuldu"
else
    print_success "Virtual environment zaten mevcut"
fi

# Activate virtual environment
source ~/air_defense_env/bin/activate

# Python dependencies
print_info "Python paketleri kuruluyor..."
pip install ultralytics opencv-python numpy
pip install Jetson.GPIO 
pip install rclpy sensor-msgs-py

print_success "Python paketleri kurulumu tamamlandı"

# GPIO permissions
print_info "GPIO izinleri ayarlanıyor..."
sudo usermod -a -G gpio $USER
sudo usermod -a -G i2c $USER  
sudo usermod -a -G spi $USER
sudo chmod 666 /dev/gpiomem

print_success "GPIO izinleri ayarlandı"

# ROS2 environment setup
print_info "ROS2 environment ayarlanıyor..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source ~/air_defense_env/bin/activate" ~/.bashrc; then
    echo "source ~/air_defense_env/bin/activate" >> ~/.bashrc
fi

print_success "Environment ayarları tamamlandı"

# Workspace build
if [ -f "ros2_air_defense/package.xml" ]; then
    print_info "ROS2 workspace build ediliyor..."
    source /opt/ros/humble/setup.bash
    colcon build --packages-select ros2_air_defense
    
    if [ -f "install/setup.bash" ]; then
        print_success "Workspace build tamamlandı"
        echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
    else
        print_error "Build başarısız!"
        exit 1
    fi
else
    print_warning "ROS2 package bulunamadı. GitHub'dan clone ettikten sonra bu scripti çalıştırın."
fi

# Test hardware availability
print_info "Hardware kontrolü yapılıyor..."

# Camera test
if ls /dev/video* 1> /dev/null 2>&1; then
    print_success "Camera cihazları bulundu: $(ls /dev/video*)"
else
    print_warning "Camera cihazı bulunamadı"
fi

# GPIO test
if [ -c /dev/gpiomem ]; then
    print_success "GPIO erişimi mevcut"
else
    print_warning "GPIO erişimi bulunamadı"
fi

# Final instructions
echo ""
echo "🎉 Kurulum tamamlandı!"
echo ""
print_info "Sistemi restart etmeniz önerilir:"
echo "sudo reboot"
echo ""
print_info "Restart sonrası test komutları:"
echo "cd ~/ROS_test"
echo "source install/setup.bash" 
echo "ros2 launch ros2_air_defense air_defense_system.launch.py simulation_mode:=true"
echo ""
print_info "Hardware test için:"
echo "ros2 launch ros2_air_defense air_defense_system.launch.py enable_gpio:=true"

print_success "Kurulum scripti tamamlandı! 🚀" 