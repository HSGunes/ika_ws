# IKA Mapping Package

Bu paket IKA robotu için kapsamlı bir SLAM (Simultaneous Localization and Mapping) ve navigasyon sistemi sağlar.

## Özellikler

- **SLAM Toolbox**: Gerçek zamanlı harita oluşturma
- **Nav2**: Tam navigasyon sistemi
- **Robot Localization**: EKF tabanlı konum tahmini
- **IMU Filter**: Madgwick filtre ile IMU verisi işleme
- **Teleop**: Klavye ile robot kontrolü
- **RViz Konfigürasyonları**: Görselleştirme için hazır konfigürasyonlar

## Kurulum

### Gereksinimler

```bash
# ROS2 Humble bağımlılıkları
sudo apt install -y ros-humble-slam-toolbox \
                   ros-humble-nav2-bringup \
                   ros-humble-robot-localization \
                   ros-humble-imu-filter-madgwick \
                   ros-humble-teleop-twist-keyboard \
                   ros-humble-tf2-tools \
                   python3-numpy
```

### Build

```bash
# Workspace'de build et
colcon build --packages-select ika_mapping

# Environment'ı source et
source install/setup.bash
```

## Kullanım

### SLAM Modu

Harita oluşturmak için:

```bash
# SLAM başlat
ros2 launch ika_mapping ika_slam.launch.py

# Teleop ile robotu kontrol et (yeni terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Navigasyon Modu

Kaydedilmiş harita ile navigasyon için:

```bash
# Navigasyon başlat
ros2 launch ika_mapping ika_navigation.launch.py map:=/path/to/map.yaml

# RViz'de 2D Pose Estimate ile başlangıç pozisyonu belirle
# RViz'de 2D Nav Goal ile hedef belirle
```

### Manuel Kontrol

```bash
# Teleop node'u başlat
ros2 run ika_mapping ika_teleop_node
```

## Konfigürasyon Dosyaları

### Nav2 Parametreleri
- `config/nav2_params.yaml`: Nav2 navigasyon parametreleri
- `config/slam_toolbox_params.yaml`: SLAM Toolbox parametreleri
- `config/robot_localization.yaml`: Robot localization parametreleri
- `config/imu_filter.yaml`: IMU filtre parametreleri

### RViz Konfigürasyonları
- `config/ika_slam.rviz`: SLAM için RViz konfigürasyonu
- `config/ika_navigation.rviz`: Navigasyon için RViz konfigürasyonu

## Launch Dosyaları

### ika_slam.launch.py
SLAM sistemi için ana launch dosyası:
- Robot State Publisher
- Joint State Publisher
- Robot Localization (EKF)
- IMU Filter
- SLAM Toolbox
- Teleop (opsiyonel)
- RViz (opsiyonel)

### ika_navigation.launch.py
Navigasyon sistemi için ana launch dosyası:
- Robot State Publisher
- Joint State Publisher
- Robot Localization (EKF)
- IMU Filter
- Map Server
- AMCL
- Nav2 Bringup
- RViz (opsiyonel)

## Python Node'ları

### ika_slam_node.py
- Laser scan verilerini işler
- Harita oluşturur
- Robot yolunu takip eder

### ika_navigation_node.py
- Hedef belirleme
- Yol planlama
- Navigasyon kontrolü

### ika_localization_node.py
- Konum tahmini
- Odometry yayınlama
- TF dönüşümleri

### ika_teleop_node.py
- Klavye kontrolü
- Hız ayarlama
- Güvenli durdurma

## Topic'ler

### Subscribers
- `/scan`: Laser scan verisi
- `/imu/data`: IMU verisi
- `/cmd_vel`: Hız komutları
- `/goal_pose`: Navigasyon hedefi

### Publishers
- `/map`: Oluşturulan harita
- `/path`: Robot yolu
- `/odom`: Odometry verisi
- `/pose`: Konum verisi
- `/cmd_vel`: Hız komutları

## TF Frame'leri

- `map` → `odom` → `base_link`
- `base_link` → `lidar_laser_frame`
- `base_link` → `imu_link`
- `base_link` → `camera_link`

## Harita Kaydetme

SLAM tamamlandıktan sonra haritayı kaydetmek için:

```bash
# Harita kaydet
ros2 run nav2_map_server map_saver_cli -f ~/ika_map

# Bu komut iki dosya oluşturur:
# - ika_map.pgm (harita görüntüsü)
# - ika_map.yaml (harita meta verisi)
```

## Sorun Giderme

### Yaygın Sorunlar

1. **TF Hatası**: Robot state publisher'ın çalıştığından emin olun
2. **Laser Scan Hatası**: Lidar'ın doğru topic'i yayınladığını kontrol edin
3. **IMU Hatası**: IMU verilerinin doğru frame'de olduğunu kontrol edin

### Debug

```bash
# TF ağacını görüntüle
ros2 run tf2_tools view_frames

# Topic'leri listele
ros2 topic list

# Topic verilerini izle
ros2 topic echo /scan
ros2 topic echo /odom
```

## Katkıda Bulunma

1. Fork yapın
2. Feature branch oluşturun
3. Değişikliklerinizi commit edin
4. Pull request gönderin

## Lisans

Bu proje MIT lisansı altında lisanslanmıştır.

## İletişim

Sorularınız için: gunes@todo.todo 