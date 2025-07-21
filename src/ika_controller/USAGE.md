# Ika Controller Kullanım Kılavuzu

## Hızlı Başlangıç

### 1. Paketi Derleme
```bash
cd ~/ika_ws
colcon build --packages-select ika_controller
source install/setup.bash
```

### 2. Controller'ı Başlatma
```bash
# Basit controller (sadece node'lar)
ros2 launch ika_controller controller.launch.py

# Tam controller (ROS2 Control ile)
ros2 launch ika_controller ika_controllers.launch.py
```

### 3. Rover'ı Kontrol Etme
```bash
# Teleop ile kontrol
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Manuel komut gönderme
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Kontrol Komutları

### İleri Gitme
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Geri Gitme
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Sola Dönme
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

### Sağa Dönme
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
```

### Durma
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Topic'leri İzleme

### Controller Çıkışlarını İzleme
```bash
# Direksiyon komutları
ros2 topic echo /front_steering_controller/commands
ros2 topic echo /rear_steering_controller/commands

# Tekerlek hız komutları
ros2 topic echo /left_wheels_controller/commands
ros2 topic echo /right_wheels_controller/commands
```

### Joint State'leri İzleme
```bash
ros2 topic echo /joint_states
```

## Parametreleri Ayarlama

### Runtime'da Parametre Değiştirme
```bash
# Maksimum hızı değiştirme
ros2 param set /ika_controller linear_velocity_max 3.0

# Maksimum açısal hızı değiştirme
ros2 param set /ika_controller angular_velocity_max 2.0
```

### Parametreleri Listeleme
```bash
ros2 param list /ika_controller
ros2 param list /ika_bridge
```

## Gazebo ile Kullanım

### 1. Gazebo'da Ika'yı Başlatma
```bash
ros2 launch ika_description gazebo.launch.py
```

### 2. Controller'ı Başlatma
```bash
ros2 launch ika_controller ika_controllers.launch.py
```

### 3. Kontrol Etme
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Sorun Giderme

### Controller Başlamıyorsa
1. Paket derlendi mi kontrol edin: `colcon build --packages-select ika_controller`
2. Setup dosyası source edildi mi: `source install/setup.bash`
3. ROS2 Control yüklü mü kontrol edin: `ros2 pkg list | grep controller`

### Tekerlekler Dönmüyorsa
1. Joint state'leri kontrol edin: `ros2 topic echo /joint_states`
2. Controller komutlarını kontrol edin: `ros2 topic echo /left_wheels_controller/commands`
3. URDF'teki joint isimlerini kontrol edin

### Direksiyon Çalışmıyorsa
1. Direksiyon komutlarını kontrol edin: `ros2 topic echo /front_steering_controller/commands`
2. Joint limitlerini kontrol edin
3. Controller parametrelerini kontrol edin

## Debug Komutları

### Node'ları Listeleme
```bash
ros2 node list
```

### Topic'leri Listeleme
```bash
ros2 topic list
```

### Service'leri Listeleme
```bash
ros2 service list
```

### Log'ları İzleme
```bash
ros2 run ika_controller ika_controller --ros-args --log-level debug
ros2 run ika_controller ika_bridge --ros-args --log-level debug
``` 