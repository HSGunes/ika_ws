# Sensör Füzyonu Kılavuzu

Bu kılavuz, RF2O Lazer Odometri ve ZED 2i Görsel-Ataletsel Odometri (VIO) sensörlerini `robot_localization` paketi ile birleştirerek robotunuzun konum tahminini iyileştirmeyi açıklar.

## Genel Bakış

Sensör füzyonu, her iki sensörün de zayıf yönlerini kapatıp güçlü yönlerini öne çıkararak daha sağlam ve güvenilir konum tahmini sağlar:

- **RF2O**: 2D LiDAR odometrisi - X, Y pozisyonu ve Yaw açısında iyi
- **ZED 2i**: 3D Görsel-Ataletsel Odometri - Tüm 6 eksende veri sağlar
- **EKF**: Genişletilmiş Kalman Filtresi ile sensör verilerini birleştirir

## Kurulum

### 1. robot_localization Paketi

```bash
sudo apt update
sudo apt install ros-humble-robot-localization
```

### 2. Konfigürasyon Dosyaları

- `config/ekf.yaml`: EKF konfigürasyonu
- `launch/sensor_fusion.launch.py`: Tüm düğümleri başlatan launch dosyası
- `launch/test_sensor_fusion.launch.py`: Sadece EKF'yi test etmek için

## Kullanım

### Adım 1: RF2O'yu TF Yayını Kapalı Olarak Başlatın

```bash
# Terminal 1
source install/setup.bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py publish_tf:=false odom_topic_name:=/odom_rf2o
```

### Adım 2: ZED 2i'yi TF Yayını Kapalı Olarak Başlatın

```bash
# Terminal 2
source install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i publish_tf:=false
```

### Adım 3: EKF Düğümünü Başlatın

```bash
# Terminal 3
source install/setup.bash
ros2 launch ika_nav2 test_sensor_fusion.launch.py
```

### Adım 4: RViz2 ile Doğrulama

```bash
# Terminal 4
source install/setup.bash
rviz2
```

#### RViz2 Ayarları:

1. **Fixed Frame**: Sol üstteki "Global Options" → "Fixed Frame" bölümünü `odom` olarak ayarlayın
2. **TF Görüntüsü**: "Add" → "By display type" → "TF" ekleyin
3. **Odometri Görüntüleri**:
   - "Add" → "By topic" → `/odom_rf2o` konusunu `Odometry` olarak ekleyin (RF2O'nun ham verisi)
   - "Add" → "By topic" → `/zed2i/zed_node/odom` konusunu `Odometry` olarak ekleyin (ZED'in ham verisi)
   - "Add" → "By topic" → `/odom` konusunu `Odometry` olarak ekleyin (**EKF'nin birleştirilmiş çıktısı**)

## Tek Komutla Başlatma

Tüm düğümleri tek seferde başlatmak için:

```bash
source install/setup.bash
ros2 launch ika_nav2 sensor_fusion.launch.py
```

## Konfigürasyon Açıklaması

### EKF Konfigürasyonu (`ekf.yaml`)

```yaml
# RF2O Lazer Odometri (2D)
odom0: /odom_rf2o
odom0_config: [true,  true,  false,   # [X, Y, Z] pozisyonları
               false, false, true,    # [Roll, Pitch, Yaw] açıları
               true,  true,  false,   # [Vx, Vy, Vz] lineer hızlar
               false, false, true]    # [Vroll, Vpitch, Vyaw] açısal hızlar

# ZED 2i Görsel Odometri (3D)
odom1: /zed2i/zed_node/odom
odom1_config: [true,  true,  true,    # [X, Y, Z] pozisyonları
               true,  true,  true,    # [Roll, Pitch, Yaw] açıları
               true,  true,  true,    # [Vx, Vy, Vz] lineer hızlar
               true,  true,  true]    # [Vroll, Vpitch, Vyaw] açısal hızlar
```

### Config Matrislerinin Açıklaması:

- **RF2O**: 2D LiDAR olduğu için Z, Roll ve Pitch eksenlerini `false` yaptık
- **ZED**: 3D VIO olduğu için tüm eksenleri `true` yaptık

## Sorun Giderme

### 1. TF Çakışması Hatası

Eğer TF çakışması hatası alırsanız:
- RF2O ve ZED'in `publish_tf` parametrelerinin `false` olduğundan emin olun
- Sadece EKF'nin TF yayınladığından emin olun

### 2. Sensör Verisi Gelmiyor

```bash
# Sensör verilerini kontrol edin
ros2 topic list | grep odom
ros2 topic echo /odom_rf2o
ros2 topic echo /zed2i/zed_node/odom
```

### 3. EKF Çıktısı Kontrolü

```bash
# EKF'nin filtrelenmiş çıktısını kontrol edin
ros2 topic echo /odom
```

## Performans İyileştirmeleri

### 1. IMU Verisi Ekleme

ZED'in IMU verisini de eklemek için `ekf.yaml` dosyasındaki IMU bölümünü açın:

```yaml
# IMU 0: ZED 2i IMU Verisi
imu0: /zed2i/zed_node/imu/data
imu0_config: [false, false, false,
              true,  true,  true,
              false, false, false,
              true,  true,  true,
              true,  true,  true]
imu0_differential: false
```

### 2. Gürültü Parametreleri

Sensörlerin güvenilirliğini ayarlamak için gürültü parametrelerini değiştirebilirsiniz:

```yaml
# Gürültü parametreleri (isteğe bağlı)
odom0_nodelay: 0.0
odom1_nodelay: 0.0
```

## Beklenen Sonuçlar

Robotunuzu hareket ettirdiğinizde:

1. `/odom` çıktısının diğer ham verilere göre daha kararlı olduğunu göreceksiniz
2. Bir sensörün zorlandığı durumlarda (örn. ZED için dokusuz duvar) diğer sensörün açığı kapattığını göreceksiniz
3. Filtrelenmiş çıktının daha az titreşimli ve tutarlı olduğunu göreceksiniz

Bu kurulumla robotunuzun konumlandırma yeteneği ciddi anlamda artacaktır!
