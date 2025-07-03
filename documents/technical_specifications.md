# Teknik Spesifikasyonlar ve Sistem Mimarisi

## Sistem Mimarisi

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Jetson Nano   │◄──►│   Kamera        │    │   YOLO Model    │
│     JNX30D      │    │   (USB/CSI)     │    │   (Detection)   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                                              │
         ▼                                              ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Motor Driver   │    │ Coordinate      │    │ Target Manager  │
│  (GPIO Control) │    │ Transformer     │    │ (Priority/Path) │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Stepper Motors  │    │ Shooting        │    │      GUI        │
│ (Base+Elevation)│    │ Controller      │    │   Interface     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## Motor Spesifikasyonları

### Stepper Motor Parametreleri
- **Tip**: Bipolar Stepper Motor
- **Step Angle**: 1.8° (200 steps/revolution)
- **Current**: 3.0A
- **Power**: 12.5W
- **Microstepping**: 1/8 veya 1/16 (daha hassas pozisyonlama için)

### Koordinat Sistemi

#### Base Motor (Azimuth - Pan)
- **Çalışma Aralığı**: 0° - 360° (veya sınırlı açı aralığı)
- **Home Position**: 0° (düz ileri)
- **Pozitif Yön**: Saat yönü (yukarıdan bakıldığında)

#### Elevation Motor (Tilt)
- **Çalışma Aralığı**: -90° - +90°
- **Home Position**: 0° (yatay)
- **Pozitif Yön**: Yukarı hareket

## Görüntü İşleme Pipeline

### YOLO Detection Parameters
```python
CONFIDENCE_THRESHOLD = 0.7
NMS_THRESHOLD = 0.4
INPUT_SIZE = (416, 416)  # YOLOv4 için
CLASS_NAMES = ['red_balloon', 'blue_balloon']
```

### Pixel-to-Angle Dönüşümü

#### Kamera Kalibrasyonu Parametreleri
```python
# Kamera FOV (Field of View)
HORIZONTAL_FOV = 60  # derece
VERTICAL_FOV = 45    # derece

# Görüntü çözünürlüğü
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# Merkez pixel koordinatları
CENTER_X = IMAGE_WIDTH // 2
CENTER_Y = IMAGE_HEIGHT // 2
```

#### Dönüşüm Algoritması
```python
def pixel_to_angle(pixel_x, pixel_y):
    # Pixel'i merkeze göre normalize et
    dx = pixel_x - CENTER_X
    dy = pixel_y - CENTER_Y
    
    # Açıya dönüştür
    pan_angle = (dx / CENTER_X) * (HORIZONTAL_FOV / 2)
    tilt_angle = -(dy / CENTER_Y) * (VERTICAL_FOV / 2)  # Y ekseni ters
    
    return pan_angle, tilt_angle
```

## Shooting Control Algorithm

### Scan-Plan-Execute İmplementasyonu

#### 1. Scanning Phase
```python
def scan_targets():
    # Home pozisyonuna git
    move_to_home()
    
    # YOLO detection başlat
    detected_targets = []
    frame = capture_frame()
    
    # YOLO inference
    detections = yolo_model.detect(frame)
    
    for detection in detections:
        if detection.class_name == 'red_balloon':
            target = {
                'bbox': detection.bbox,
                'confidence': detection.confidence,
                'pixel_coords': detection.center,
                'motor_angles': pixel_to_angle(*detection.center)
            }
            detected_targets.append(target)
    
    return detected_targets
```

#### 2. Path Planning
```python
def plan_shooting_path(targets):
    # Hedefleri mesafeye göre sırala (en yakından başla)
    current_position = get_current_motor_position()
    
    sorted_targets = sorted(targets, key=lambda t: 
        calculate_motor_distance(current_position, t['motor_angles']))
    
    return sorted_targets
```

#### 3. Execution Phase
```python
def execute_shooting(target_sequence):
    for target in target_sequence:
        # Hedefe yönel
        move_to_target(target['motor_angles'])
        
        # Pozisyon doğrulaması
        if verify_target_position(target):
            # Ateş et
            fire_shot()
            time.sleep(0.5)  # Stabilization delay
        
        # Sonraki hedefe geç
    
    # Home pozisyonuna dön
    move_to_home()
```

## Motor Control Implementation

### Stepper Motor Driver
```python
class StepperMotorController:
    def __init__(self, step_pin, dir_pin, enable_pin, steps_per_rev=200):
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self.steps_per_rev = steps_per_rev
        self.current_position = 0
        
    def move_to_angle(self, target_angle, speed=1000):
        steps_needed = int((target_angle / 360) * self.steps_per_rev)
        self.move_steps(steps_needed - self.current_position, speed)
        
    def move_steps(self, steps, speed):
        # Direction kontrolü
        if steps > 0:
            GPIO.output(self.dir_pin, GPIO.HIGH)
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)
            steps = abs(steps)
        
        # Step pulse'ları gönder
        delay = 1.0 / (2 * speed)  # microseconds
        for _ in range(steps):
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(delay)
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(delay)
        
        self.current_position += steps
```

## Safety and Error Handling

### Güvenlik Kontrolü
- **Limit Switch**: Mekanik limitler
- **Software Limits**: Motor açı sınırları
- **Emergency Stop**: Acil durdurma protokolü
- **Friendly Fire Protection**: Mavi hedef koruma

### Error Recovery
```python
def safe_shutdown():
    # Motorları durdur
    disable_motors()
    
    # Valf sistemini kapat
    close_firing_valve()
    
    # Home pozisyonuna dön
    emergency_home_return()
```

## Performance Metrics

### Hedef Performans
- **Detection Accuracy**: >95% (kırmızı/mavi ayrımı)
- **Positioning Accuracy**: ±2° motor hassasiyeti
- **Total Response Time**: 2-3 saniye
- **Shooting Precision**: %90+ hit rate

### Test Senaryoları
1. **Single Target**: Tek hedef hızlı vurma
2. **Multiple Targets**: 3-5 hedef sıralı vurma
3. **Mixed Targets**: Kırmızı/mavi karışık test
4. **Moving Targets**: Hafif salınan hedefler
5. **Different Distances**: Yakın/uzak hedef testleri 