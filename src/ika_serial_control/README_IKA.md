# IKA Serial Control Package

Bu paket, IKA robotu için ROS2 tabanlı serial controller sağlar. IKA robotu 6 tekerlekli bir platformdur ve 2 adet 360 derece dönen ön tekerleği vardır.

## Özellikler

- **6 Tekerlek Kontrolü**: Sol ve sağ tarafta 3'er tekerlek
- **360° Direksiyon**: Ön tekerlekler stepper motor ile döndürülür
- **RC ve Serial Kontrol**: Hem RC kumanda hem de ROS2 serial komut desteği
- **Güvenlik Özellikleri**: Komut timeout ve hız sınırlamaları
- **Gerçek Zamanlı Durum**: Tekerlek hızları ve direksiyon açısı takibi

## Donanım Gereksinimleri

### Arduino Mega 2560
- **DC Motor Sürücüleri**: 6 adet (her tekerlek için)
- **Stepper Motor Sürücüleri**: 2 adet (direksiyon için)
- **RC Alıcı**: 4 kanal (opsiyonel)

### Pin Bağlantıları

#### DC Motor Pinleri (6 Tekerlek)
```
Sol Ön Tekerlek:   LPWM1=53, RPWM1=52, LEN1=51, REN1=50
Sol Orta Tekerlek: LPWM2=49, RPWM2=48, LEN2=47, REN2=46
Sol Arka Tekerlek: LPWM3=45, RPWM3=44, LEN3=43, REN3=42
Sağ Ön Tekerlek:   LPWM4=41, RPWM4=40, LEN4=39, REN4=38
Sağ Orta Tekerlek: LPWM5=37, RPWM5=36, LEN5=35, REN5=34
Sağ Arka Tekerlek: LPWM6=33, RPWM6=32, LEN6=31, REN6=30
```

#### Stepper Motor Pinleri (Direksiyon)
```
Sol Ön Direksiyon: STEP1=28, DIR1=29, ENA1=12
Sağ Ön Direksiyon: STEP2=26, DIR2=27, ENA2=11
```

#### RC Kanal Pinleri (Opsiyonel)
```
CH1 (Direksiyon): A1
CH2 (İleri/Geri): A2
CH6 (Fren): A4
CH7 (Ateş): A3
```

## Kurulum

### 1. Arduino Kodu Yükleme
```bash
# Arduino IDE'de ika_arduino_controller.ino dosyasını açın
# Board: Arduino Mega 2560 seçin
# Port: Arduino'nun bağlı olduğu portu seçin
# Upload edin
```

### 2. ROS2 Paket Kurulumu
```bash
# Workspace'de build edin
cd ~/ika_ws
colcon build --packages-select ika_serial_control

# Source edin
source install/setup.bash
```

## Kullanım

### 1. Temel Çalıştırma
```bash
# IKA controller'ı başlatın
ros2 launch ika_serial_control ika_control.launch.py

# Veya parametrelerle
ros2 launch ika_serial_control ika_control.launch.py serial_port:=/dev/ttyUSB0
```

### 2. Manuel Test
```bash
# Teleop ile test edin
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Veya geometry_msgs/Twist mesajı gönderin
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3. Durum İzleme
```bash
# Robot durumunu izleyin
ros2 topic echo /ika_status
ros2 topic echo /current_steer_angle
ros2 topic echo /current_speed
ros2 topic echo /wheel_speeds
```

## Parametreler

### Temel Parametreler
- `serial_port`: Arduino bağlantı portu (varsayılan: /dev/ttyUSB0)
- `baud_rate`: Serial baud rate (varsayılan: 115200)
- `max_linear_speed`: Maksimum PWM değeri (varsayılan: 170)
- `max_angular_speed`: Maksimum direksiyon açısı (varsayılan: 60°)

### Scaling Parametreleri
- `linear_scale_factor`: Linear hız ölçekleme (varsayılan: 85.0)
- `angular_scale_factor`: Angular hız ölçekleme (varsayılan: 60.0)
- `min_pwm_threshold`: Minimum PWM eşiği (varsayılan: 10)
- `max_cmd_linear`: Maksimum kabul edilen linear komut (varsayılan: 2.0 m/s)
- `max_cmd_angular`: Maksimum kabul edilen angular komut (varsayılan: 1.5 rad/s)

### IKA Özel Parametreleri
- `enable_steering`: Direksiyon kontrolü aktif mi (varsayılan: true)
- `steering_sensitivity`: Direksiyon hassasiyeti (varsayılan: 1.0)
- `wheel_base`: Tekerlekler arası mesafe (varsayılan: 0.8 m)
- `track_width`: Tekerlek izi genişliği (varsayılan: 0.78 m)

## Komut Formatı

### Serial Komut Formatı
```
STEER:angle,SPEED:speed
```
Örnek:
```
STEER:30.0,SPEED:100    # 30° sağa dön, 100 PWM ile ileri git
STEER:-15.0,SPEED:-50   # 15° sola dön, 50 PWM ile geri git
STEER:0.0,SPEED:0       # Dur
```

### ROS2 Topic Formatı
```bash
# geometry_msgs/Twist mesajı
linear.x:  İleri/geri hız (m/s)
angular.z: Dönüş hızı (rad/s)
```

## Güvenlik Özellikleri

1. **Komut Timeout**: 500ms komut gelmezse otomatik durma
2. **Hız Sınırlaması**: Maksimum 2.0 m/s linear, 1.5 rad/s angular
3. **PWM Sınırlaması**: Maksimum 170 PWM değeri
4. **Minimum Threshold**: Çok küçük komutları filtreleme

## Sorun Giderme

### Serial Bağlantı Sorunları
```bash
# Port izinlerini kontrol edin
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0

# Port varlığını kontrol edin
dmesg | grep tty
```

### Arduino Debug
```bash
# Arduino Serial Monitor'ü açın (115200 baud)
# Debug mesajlarını kontrol edin
```

### ROS2 Debug
```bash
# Log seviyesini artırın
ros2 launch ika_serial_control ika_control.launch.py --ros-args --log-level debug
```

## Katkıda Bulunma

1. Fork edin
2. Feature branch oluşturun (`git checkout -b feature/yeni-ozellik`)
3. Commit edin (`git commit -am 'Yeni özellik eklendi'`)
4. Push edin (`git push origin feature/yeni-ozellik`)
5. Pull Request oluşturun

## Lisans

Bu proje Apache 2.0 lisansı altında lisanslanmıştır.
