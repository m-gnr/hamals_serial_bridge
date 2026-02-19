# HAMALS Serial Bridge

> ROS 2 ile gömülü MCU firmware arasında güvenli ve katmanlı seri haberleşme köprüsü.
> 

hamals_serial_bridge, diferansiyel sürüşlü mobil robotlarda ROS 2 ekosistemi ile mikrodenetleyici firmware’i arasında kontrollü ve güvenli bağlantı sağlayan modüler bir middleware katmanıdır.

## Amaç

Bu paket iki farklı dünyayı birbirine bağlar

### ROS 2 Tarafı

- Nav2
- Teleop
- EKF / robot_localization
- RViz
- TF sistemi

### MCU Tarafı

- Encoder ISR
- IMU polling
- PID kontrol
- odometri hesaplama

hamals_serial_bridge, ROS 2 ile firmware arasında güvenli bir sistem sınırı (system boundary) oluşturur ve veri akışını kontrollü şekilde yönetir.

## Mimari

```text
[ ROS Graph Layer ]
        |
[ Bridge Orchestrator (serial_node) ]
        |
[ Transport & Protocol Layer ]
        |
[ Embedded Firmware ]
```

## Tasarım İlkeleri

- Tek sorumluluk prensibi
- Katmanlı mimari
- Config-driven yapı
- Dead-man güvenlik mekanizması
- Thread izolasyonu

## Veri Akışı

### **ROS → MCU**

```text
Twist → encode_cmd() → serial.write()
```

/cmd_vel mesajı MCU protokolüne çevirerek seri hat üzerinden gönderilir.

### MCU → ROS

```text
serial.read → parser.push() → decode_line() → publish(Odometry)
```

MCU’dan gelen odometri verisi ayrıştırılır ve ROS nav_msgs/Odometry mesajına dönüştürülür.

## Paket Yapısı

```python
hamals_serial_bridge/
├── config/serial_bridge.yaml
├── hamals_serial_bridge/
│   ├── parser.py
│   ├── protocol.py
│   ├── serial_node.py
│   ├── utils.py
├── launch/serial_bridge.launch.py
├── test/test_parser.py
```

## Konfigürasyon

```yaml
/**:
  ros__parameters:
    port: /dev/ttyACM0
    baudrate: 115200
    timeout_ms: 50

    cmd_vel_topic: /cmd_vel
    odom_topic: /odom
    odom_pub_hz: 50

    reset_on_startup: true
    cmd_vel_timeout_ms: 500
    debug: true
```

## Güvenlik Mekanizması

Belirli süre içinde /cmd_vel alınmazsa:

```text
CMD 0.000 0.000
```

gönderilir ve robot durdurulur.

Bu sayede kontrolsüz hareket (runaway) durumu engellenir.

- WiFi kopması
- Teleop çökmesi
- Nav2 hatası
- Node kapanması

## Thread Modeli

- ROS callback’leri ana thread’de çalışır
- Serial RX ayrı daemon thread’dedir
- Bloklayıcı okuma ROS tarafını kilitlemez

## Seri Protokol

**ROS → MCU**

```text
CMD <linear_velocity> <angular_velocity>\n
```

Örnek:

```text
CMD 0.200 0.000
```

### MCU → ROS

```text
odom,x,y,yaw,v,w
```

Örnek:

```text
odom,1.23,0.45,0.12,0.20,0.00
```

## Debug Modu

debug: true ise:

- RX paket sayısı
- TX paket sayısı
- Yayınlanan odom sayısı
- Dead-man durumu
- Son cmd_vel
- Son odom

1 saniyede bir terminalde gösterilir.

## Kullanım

Build:

```text
colcon build --packages-select hamals_serial_bridge
source install/setup.bash
```

Launch:

```text
ros2 launch hamals_serial_bridge serial_bridge.launch.py
```
