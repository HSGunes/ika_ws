# Ika Controller Package

Bu paket, 6 tekerlekli Ika rover'ı için controller sağlar. Rover'ın 4 tekerleği 360 derece dönebilir (ön ve arka tekerler) ve 6 tekerleği de sürüş tekerleğidir.

## Özellikler

- **6 Tekerlek Sürüş**: Sol ve sağ tarafta 3'er tekerlek
- **4 Tekerlek Direksiyon**: Ön ve arka tekerler 360 derece dönebilir
- **Twist Kontrolü**: `/cmd_vel` topic'i üzerinden standart ROS2 twist mesajları ile kontrol
- **Hız Sınırlaması**: Maksimum hız ve ivme sınırları
- **Gazebo Uyumluluğu**: ROS2 Control ile Gazebo simülasyonu desteği

## Tekerlek Konfigürasyonu

### Sürüş Tekerlekleri (6 adet)
- `sol_on_teker` - Sol ön tekerlek
- `sol_orta_teker` - Sol orta tekerlek  
- `sol_arka_teker` - Sol arka tekerlek
- `sag_on_teker` - Sağ ön tekerlek
- `sag_orta_teker` - Sağ orta tekerlek
- `sag_arka_teker` - Sağ arka tekerlek

### Direksiyon Tekerlekleri (4 adet)
- `sol_on_360` - Sol ön direksiyon
- `sag_on_360` - Sağ ön direksiyon
- `sol_arka_360` - Sol arka direksiyon
- `sag_arka_360` - Sağ arka direksiyon

## Kullanım

### Paketi Derleme
```bash
cd ~/ika_ws
colcon build --packages-select ika_controller
source install/setup.bash
```

### Controller'ı Başlatma
```bash
# Tam controller başlatma (ROS2 Control ile)
ros2 launch ika_controller ika_controllers.launch.py

# Sadece controller node'ları
ros2 launch ika_controller controller.launch.py
```

### Rover'ı Kontrol Etme
```bash
# Teleop ile kontrol
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Twist mesajı gönderme
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Konfigürasyon

### Controller Parametreleri
- `linear_velocity_max`: Maksimum doğrusal hız (m/s)
- `angular_velocity_max`: Maksimum açısal hız (rad/s)
- `linear_acceleration_max`: Maksimum doğrusal ivme (m/s²)
- `angular_acceleration_max`: Maksimum açısal ivme (rad/s²)
- `wheelbase`: Tekerlek arası mesafe (m)
- `track_width`: Sol-sağ tekerlek arası mesafe (m)
- `wheel_radius`: Tekerlek yarıçapı (m)
- `max_steering_angle`: Maksimum direksiyon açısı (derece)

### Topic'ler

#### Giriş Topic'leri
- `/cmd_vel` (geometry_msgs/Twist): Standart ROS2 twist komutları

#### Çıkış Topic'leri
- `/front_steering_controller/commands` (std_msgs/Float64MultiArray): Ön direksiyon komutları
- `/rear_steering_controller/commands` (std_msgs/Float64MultiArray): Arka direksiyon komutları
- `/left_wheels_controller/commands` (std_msgs/Float64MultiArray): Sol tekerlek hız komutları
- `/right_wheels_controller/commands` (std_msgs/Float64MultiArray): Sağ tekerlek hız komutları

## Test

```bash
# Test çalıştırma
ros2 test ika_controller
```

## Gazebo ile Kullanım

Gazebo simülasyonunda kullanmak için:

1. Ika description paketini yükleyin
2. Gazebo world'ünü başlatın
3. Controller'ı başlatın:

```bash
# Gazebo'da ika'yı başlatma
ros2 launch ika_description gazebo.launch.py

# Yeni terminal'de controller'ı başlatma
ros2 launch ika_controller ika_controllers.launch.py
```

## Katkıda Bulunma

1. Fork yapın
2. Feature branch oluşturun (`git checkout -b feature/amazing-feature`)
3. Commit yapın (`git commit -m 'Add amazing feature'`)
4. Push yapın (`git push origin feature/amazing-feature`)
5. Pull Request açın

## Lisans

Bu proje MIT lisansı altında lisanslanmıştır. 