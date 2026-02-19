# HAMALS Serial Bridge

> ROS2 ile gömülü MCU firmware arasında güvenli ve katmanlı seri haberleşme köprüsü.
> 

hamals_serial_bridge, diferansiyel sürüşlü mobil robotlarda ROS 2 ekosistemi ile mikrodenetleyici firmware’i arasında kontrollü ve güvenli bağlantı sağlayan modüler bir middleware katmanıdır.

## Amaç

Bu paket iki farklı dünyayı birbirine bağlar

### ROS 2 Tarafı

- NAV2
- Teleop
- EKF / robot_localization
- RViz
- TF sistemi

### MCU Tarafı

- Encoder ISR
- IMU polling
- PID kontrol
- Odometry hesaplama
- Watchdog fail-safe

Serial bridge, ROS ile firmware arasında **system boundary** oluşturur ve veri akışını doğrulamalı (checksum’lu) şekilde yönetir.

# Mimari

```
[ ROS Graph Layer ]
        |
[ SerialBridge Node ]
        |
[ Framed Protocol Layer ]
        |
[ Embedded Firmware ]
```

## Tasarım İlkeleri

- Tek sorumluluk prensibi
- Katmanlı mimari
- Config-driven yapı
- Thread-safe TX (lock korumalı)
- Framed + checksum’lu protokol
- Dead-man güvenlik mekanizması
- Parser istatistikleri
- Nav2 uyumlu covariance

## Veri Akışı

### **ROS → MCU**

```
Twist → encode_cmd() → framed packet → serial.write()
```

/cmd_vel mesajı MCU protokolüne çevirerek seri hat üzerinden gönderilir.

### MCU → ROS

```
serial.read → LineParser → checksum verify → publish(Odometry)
```

MCU’dan gelen odometri verisi ayrıştırılır ve ROS nav_msg /Odometry mesajına dönüştürülür.

## Paket Yapısı

```
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
  ros__parameters:

    port: /dev/ttyACM0
    baudrate: 115200
    timeout_ms: 50

    cmd_vel_topic: /cmd_vel
    odom_topic: /odom
    odom_pub_hz: 50

    frame_id: odom
    child_frame_id: base_link

    pose_covariance: [ ... 36 elements ... ]
    twist_covariance: [ ... 36 elements ... ]

    reset_on_startup: true
    reset_pulse_ms: 100
    reset_boot_wait_ms: 1500

    cmd_vel_timeout_ms: 500
    debug: true
```

## Güvenlik Mekanizması

Belirli süre içinde /cmd_vel alınmazsa:

```yaml
CMD 0.000 0.000
```

gönderilir ve robot durdurulur.

Bu durumlarda runaway robotr engellenir.

- WiFi kopması
- Teleop çökmesi
- Nav2 hatası
- Node kapanması

## Framed Seri Protokol

**ROS → MCU**

```yaml
CMD <linear_velocity> <angular_velocity>
```

Örnek:

```
$CMD,0.200,0.000*5A
```

### MCU → ROS

```yaml
odom,x,y,yaw,v,w
```

Örnek:

```
$ODOM,1.23,0.45,0.12,0.20,0.00*3F
```

Invalid checksum türünde frame’ler parser tarafından discard edilir.

## Thread Modeli

- ROS callback ana thread
- Serial RX ayrı daemon thread
- TX lock ile thread-safe write
- Parser realign + framed recovery

## Debug Modu

debug: true ise:

- RX bytes
- RX valid frames
- RX invalid frames
- TX packets
- Odom publish count
- Dead-man state
- Son cmd_vel
- Son odom

1 saniyede bir terminalde gösterilir.

## Kullanım

Build:

```yaml
colcon build --packages-select hamals_serial_bridge
source install/setup.bash
```

Launch:

```yaml
ros2 launch hamals_serial_bridge serial_bridge.launch.py
```

## Ekosistem

Bu paket, HAMALS robot yazılım mimarisinin bir parçasıdır.

Bağlantılı olduğu firmware:

- **[hamals_firmware](https://github.com/m-gnr/hamals_firmware)**  
  Diferansiyel sürüşlü robot için MCU tabanlı gerçek zamanlı kontrol katmanı.

```
            ┌────────────────────────────┐
            │        ROS 2 Layer         │
            │  Nav2 | EKF | RViz | TF    │
            └───────────────┬────────────┘
                            │
                  hamals_serial_bridge
                            │
            ┌───────────────┴────────────┐
            │       hamals_firmware      │
            │  PID | IMU | Encoder | Odom│
            └────────────────────────────┘
```
