# RobotAksi Serial Control

ROS2 Humble ile Arduino arasında serial iletişim kurarak araç kontrolü sağlayan paket.

## Özellikler

- ✅ ROS2 Humble desteği
- ✅ Arduino ile serial iletişim
- ✅ Stepper motor ile direksiyon kontrolü
- ✅ PWM ile hız kontrolü
- ✅ Klavye ile teleop kontrolü
- ✅ Güvenlik kontrolleri
- ✅ Gerçek zamanlı durum bildirimi

## Sistem Gereksinimleri

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.8+
- pyserial kütüphanesi
- Arduino IDE (Arduino kodu için)

## Arduino Kurulumu

### Gerekli Kütüphaneler
```bash
# Arduino IDE'de Library Manager'dan yükleyin:
# - AccelStepper
```

### Donanım Bağlantıları
```
Arduino Pin Bağlantıları:
- Pin 8: STEP (Stepper Motor)
- Pin 7: DIR (Stepper Motor)
- Pin 2: PWM (Arka Motor)
```

### Arduino Kodunu Yükleme
1. `arduino_vehicle_control.ino` dosyasını Arduino IDE'de açın
2. Doğru board ve port'u seçin
3. Kodu Arduino'ya yükleyin

## ROS2 Kurulumu

### 1. Workspace'i Build Edin
```bash
cd ~/ros2_ws
colcon build --packages-select robotaksi_serial_control
source install/setup.bash
```

### 2. Gerekli Python Paketlerini Yükleyin
```bash
pip3 install pyserial
```

## Kullanım

### 1. Serial Controller'ı Başlatın
```bash
# Varsayılan ayarlarla
ros2 launch robotaksi_serial_control vehicle_control.launch.py

# Özel port ile
ros2 launch robotaksi_serial_control vehicle_control.launch.py serial_port:=/dev/ttyACM0

# Tüm parametrelerle
ros2 launch robotaksi_serial_control vehicle_control.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baud_rate:=115200 \
    max_linear_speed:=255 \
    max_angular_speed:=45.0
```

### 2. Klavye Kontrolü
```bash
# Yeni terminal açın
ros2 run robotaksi_serial_control vehicle_teleop
```

#### Klavye Kontrolleri:
- **w/s**: İleri/Geri
- **a/d**: Sol/Sağ dönüş  
- **x**: Acil dur
- **SPACE**: Dur
- **q/z**: Doğrusal hızı artır/azalt
- **e/c**: Açısal hızı artır/azalt
- **i**: Durum bilgisi
- **h**: Yardım
- **CTRL+C**: Çıkış

### 3. Programatik Kontrol
```bash
# Twist mesajı ile kontrol
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "linear: {x: 0.5, y: 0.0, z: 0.0}
   angular: {x: 0.0, y: 0.0, z: 0.3}"
```

## ROS2 Topic'ler

### Subscribed Topics
- `/cmd_vel` (geometry_msgs/Twist): Hareket komutları

### Published Topics
- `/arduino_status` (std_msgs/String): Arduino durum mesajları
- `/current_steer_angle` (std_msgs/Float32): Mevcut direksiyon açısı
- `/current_speed` (std_msgs/Int32): Mevcut hız

## Parametreler

| Parametre | Varsayılan | Açıklama |
|-----------|------------|----------|
| `serial_port` | `/dev/ttyUSB0` | Arduino serial port |
| `baud_rate` | `115200` | Serial iletişim hızı |
| `timeout` | `1.0` | Serial timeout (saniye) |
| `max_linear_speed` | `255` | Maksimum PWM değeri |
| `max_angular_speed` | `45.0` | Maksimum direksiyon açısı (derece) |

## Güvenlik Özellikleri

- **Timeout Koruması**: 2 saniye komut gelmezse araç durur
- **Hız Sınırlama**: PWM ve açı değerleri sınırlanır
- **Acil Dur**: X tuşu ile anında durdurma
- **Bağlantı Kontrolü**: Serial bağlantı kopukluğu kontrolü

## Troubleshooting

### Serial Port Problemleri
```bash
# Port'ları listeleyin
ls /dev/tty*

# Port izinlerini kontrol edin
sudo usermod -a -G dialout $USER
# Oturum açın/çıkın

# Port'u test edin
sudo chmod 666 /dev/ttyUSB0
```

### Arduino Bağlantı Problemleri
1. Arduino'nun doğru porta bağlı olduğunu kontrol edin
2. Baud rate'in doğru olduğunu kontrol edin (115200)
3. Arduino Serial Monitor'da mesajları kontrol edin

### ROS2 Node Problemleri
```bash
# Node'ları kontrol edin
ros2 node list

# Topic'leri kontrol edin  
ros2 topic list
ros2 topic echo /arduino_status

# Log'ları kontrol edin
ros2 run robotaksi_serial_control serial_controller --ros-args --log-level debug
```

## Geliştirme

### Yeni Özellik Ekleme
1. `serial_controller.py` dosyasını düzenleyin
2. Gerekirse Arduino kodunu güncelleyin
3. Package'ı yeniden build edin

### Test Etme
```bash
# Package'ı test edin
colcon test --packages-select robotaksi_serial_control

# Manuel test
ros2 run robotaksi_serial_control serial_controller
```

## Lisans

Apache-2.0

## Katkıda Bulunma

1. Fork edin
2. Feature branch oluşturun
3. Değişikliklerinizi commit edin
4. Pull request gönderin 