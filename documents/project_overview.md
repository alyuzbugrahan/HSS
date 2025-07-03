# Hava Savunma Sistemi Projesi - Genel Bakış

## Proje Amacı
Hava savunma sistemi yarışması için renk bazlı balon hedefleme ve ateşleme sistemi geliştirmek.

## Sistem Gereksinimleri

### Donanım
- **İşlemci**: Jetson Nano JNX30D
- **Görüntü İşleme**: YOLO algoritması
- **Motor Sistemi**: 2x Stepper Motor (1.8°, 3.0A, 12.5W)
  - Base Motor (Azimuth): Yatay düzlem radial hareket
  - Elevation Motor: Dikey düzlem hedefleme
- **Silah Sistemi**: Basınçlı gaz + valf kontrol (paintball tarzı)
- **Kamera**: Namlu üzerine monte

### Yazılım
- **Hedef Tanıma**: YOLO (kırmızı/mavi balon sınıflandırması)
- **Kontrol Arayüzü**: GUI ile manuel ateş kontrolü
- **Motor Kontrolü**: Stepper motor driver sistemi

## Operasyon Modeli

### Hedef Türleri
- **Düşman (Kırmızı)**: Patlatılacak hedefler
- **Dost (Mavi)**: Korunacak hedefler

### Çalışma Stratejisi: "Scan-Plan-Execute"
1. **Scan**: Home pozisyondan tüm alanı tara
2. **Plan**: Hedefleri belirle ve atış sırasını hesapla
3. **Execute**: Sıralı olarak hedefleri vur
4. **Return**: Home pozisyonuna geri dön

## Performans Hedefleri
- **Tepki Süresi**: 2-3 saniye (Precision Mode)
- **YOLO FPS**: 15-20
- **Motor Hassasiyeti**: 1.8° step resolution
- **Güvenlik**: Dost birimlere zarar vermeme 