# IMU-DDS Adapter — Architecture Diagrams

## 1. System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Hardware Layer                                    │
│                                                                         │
│   ┌───────────────────────┐                                             │
│   │  VectorNav VN-100S-CR │  ← UART 115200 baud, 50 Hz                 │
│   │   IMU Sensor          │    $VNIMU ASCII protocol                    │
│   └──────────┬────────────┘                                             │
│              │ /dev/ttyUSB0                                             │
└──────────────┼──────────────────────────────────────────────────────────┘
               │
┌──────────────▼──────────────────────────────────────────────────────────┐
│                    imu_dds_adapter (ROS 2 Node)                         │
│                                                                         │
│  ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────────┐   │
│  │  Thread A        │   │  Thread B        │   │   ROS 2 Components  │   │
│  │  Serial Reader   │   │  ROS 2 Executor  │   │                     │   │
│  │                  │   │                  │   │  Publishers:        │   │
│  │  SerialPort      │   │  Timer @ 50 Hz   │   │  /imu/data          │   │
│  │  .readline()  ──►│──►│  processQueue()  │──►│  /imu/data_raw      │   │
│  │                  │   │                  │   │  /imu/raw_ascii     │   │
│  │  [serial_mutex_] │   │  pollQuaternion()│   │  /imu/status        │   │
│  │                  │   │  [serial_mutex_] │   │                     │   │
│  └─────────────────┘   └─────────────────┘   └─────────────────────┘   │
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │  Processing Pipeline                                              │   │
│  │                                                                   │   │
│  │  NmeaParser ──► ImuFrame ──► ImuConverter ──► sensor_msgs/Imu   │   │
│  │  (parse ASCII)   (DTO)     (NED→ENU + cov)   (ROS 2 message)    │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
               │ DDS / Fast-DDS
               │ /imu/data  /imu/data_raw  /imu/raw_ascii  /imu/status
┌──────────────▼──────────────────────────────────────────────────────────┐
│                    DDS Middleware (Fast-DDS)                             │
│              sensor_msgs/Imu  ·  std_msgs/String  ·  diagnostic_msgs    │
└───────┬──────────────────────────────┬────────────────────────┬─────────┘
        │                              │                        │
┌───────▼───────┐             ┌────────▼────────┐    ┌─────────▼────────┐
│ imu_dds_monitor│             │    RViz2         │    │  Any DDS         │
│ (rqt plugin)  │             │  rviz_imu_plugin │    │  subscriber      │
│               │             │  TF display      │    │  (Wireshark,     │
│ Tab: Raw ASCII│             │  @ imu_link      │    │   rtiddsspy,     │
│ Tab: Parsed   │             └─────────────────┘    │   custom nodes)  │
│ Tab: DDS Out  │                                     └──────────────────┘
└───────────────┘
```

## 2. Threading Model

```
  /dev/ttyUSB0
       │
       ▼
┌──────────────────────────────────────────────────────────┐
│ Thread A — Serial Reader (blocking)                      │
│                                                          │
│  loop:                                                   │
│    lock(serial_mutex_)                                   │
│    line = serial_.readline()          ← blocks ≤ 500 ms  │
│    unlock(serial_mutex_)                                 │
│    if "$VNIMU" in line:                                  │
│        lock(queue_mutex_)                                │
│        frame_queue_.push(line)                           │
│        unlock(queue_mutex_)                              │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│ Thread B — ROS 2 Executor (timer @ 50 Hz = 20 ms)        │
│                                                          │
│  every 20 ms:                                            │
│    lock(queue_mutex_)                                    │
│    frame = frame_queue_.pop()                            │
│    unlock(queue_mutex_)                                  │
│                                                          │
│    frame = NmeaParser::parseVnimu(line)                  │
│                                                          │
│    every 5th tick:                                       │
│      lock(serial_mutex_)                                 │
│      serial_.write("$VNRRG,09*XX\r\n")  ← poll quat     │
│      response = serial_.readline()                       │
│      unlock(serial_mutex_)                               │
│      frame.quat = NmeaParser::parseQuaternion(response)  │
│                                                          │
│    msg = ImuConverter::convert(frame, "imu_link")        │
│    imu_pub_->publish(msg)                                │
└──────────────────────────────────────────────────────────┘

  Shared state protected by mutexes:
    serial_mutex_ → SerialPort fd_ (readline + write)
    queue_mutex_  → frame_queue_
    quat_mutex_   → quat_cache_
```

## 3. Data Flow

```
VN-100 UART Output (50 Hz):
  $VNIMU,+0.0196,-0.0023,-0.3065,-0.063,+0.045,-9.802,-0.005,+0.006,-0.016,+25.43,+101.32*64\r\n
           │        │        │       │      │      │      │      │      │      │       │
           Mag X   Mag Y   Mag Z  Acc X  Acc Y  Acc Z  Gyro X Gyro Y Gyro Z  Temp   Press
           (Gauss)                (m/s²)                (rad/s)                (°C)   (kPa)

           │
           ▼
  NmeaParser::validateChecksum()   ← XOR of bytes between $ and *
           │
           ▼
  NmeaParser::parseVnimu()         ← tokenize by comma, parse doubles
           │
           ▼
  ImuFrame {                        ← Data Transfer Object
    accel_x=-0.063, accel_y=+0.045, accel_z=-9.802
    gyro_x=-0.005,  gyro_y=+0.006,  gyro_z=-0.016
    mag_x=+0.0196,  mag_y=-0.0023,  mag_z=-0.3065
    temperature=25.43, pressure=101.32
    qx,qy,qz,qw (from poll, every 5th frame)
  }
           │
           ▼
  ImuConverter::convert()
    NED→ENU: q_enu = q_rot ⊗ q_ned    (q_rot = 180° around (1/√2, 1/√2, 0))
    covariance:
      orientation:        diag(1e-4)    rad²
      angular_velocity:   diag(1.2e-7)  rad²/s²
      linear_acceleration: diag(4e-5)   m²/s⁴
           │
           ▼
  sensor_msgs/Imu {
    header: { stamp: <ROS time>, frame_id: "imu_link" }
    orientation: { x, y, z, w }  (ENU)
    angular_velocity: { x, y, z }
    linear_acceleration: { x, y, z }
    orientation_covariance: [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]
    ...
  }
           │
           ▼ DDS / Fast-DDS
  /imu/data  (sensor_msgs/Imu)    ← filtered: orientation valid
  /imu/data_raw (sensor_msgs/Imu) ← always published
  /imu/raw_ascii (std_msgs/String) ← raw NMEA line for debugging
  /imu/status (diagnostic_msgs/DiagnosticStatus) ← frames, drops, errors
```

## 4. rqt Plugin UI Mockup

```
┌─────────────────────────────────────────────────────────────────────┐
│  IMU DDS Monitor                                            [X] [□] │
├─────────────────────────────────────────────────────────────────────┤
│ [Raw ASCII] [Parsed Fields] [DDS Output]                            │
├─────────────────────────────────────────────────────────────────────┤
│ Raw ASCII tab:                                                      │
│ ┌─────────────────────────────────────────────────────────────────┐ │
│ │ $VNIMU,+0.0196,-0.0023,-0.3065,-0.063,+0.045,-9.802,...*64    │ │
│ │ $VNIMU,+0.0197,-0.0024,-0.3066,-0.062,+0.046,-9.801,...*61    │ │
│ │ $VNIMU,+0.0195,-0.0022,-0.3064,-0.064,+0.044,-9.803,...*63    │ │
│ │ ...  (scrolling, max 50 lines)                                  │ │
│ └─────────────────────────────────────────────────────────────────┘ │
├─────────────────────────────────────────────────────────────────────┤
│ Parsed Fields tab:                                                  │
│ ┌───────────────────┬────────────────────────────────────────────┐ │
│ │ Campo             │ Valor                                      │ │
│ ├───────────────────┼────────────────────────────────────────────┤ │
│ │ accel_x [m/s2]    │ -0.063000                                  │ │
│ │ accel_y [m/s2]    │  0.045000                                  │ │
│ │ accel_z [m/s2]    │ -9.802000                                  │ │
│ │ gyro_x [rad/s]    │ -0.005000                                  │ │
│ │ gyro_y [rad/s]    │  0.006000                                  │ │
│ │ gyro_z [rad/s]    │ -0.016000                                  │ │
│ │ quat_x            │  0.578577                                  │ │
│ │ quat_y            │ -0.815619                                  │ │
│ │ quat_z            │ -0.003497                                  │ │
│ │ quat_w            │ -0.001703                                  │ │
│ │ frame_id          │ imu_link                                   │ │
│ └───────────────────┴────────────────────────────────────────────┘ │
├─────────────────────────────────────────────────────────────────────┤
│ DDS Output tab:                                                     │
│ ┌───────────────────┬────────────────────────────────────────────┐ │
│ │ Campo DDS         │ Valor                                      │ │
│ ├───────────────────┼────────────────────────────────────────────┤ │
│ │ topic             │ /imu/data                                  │ │
│ │ seq               │ 1950                                       │ │
│ │ stamp.sec         │ 1779549096                                 │ │
│ │ stamp.nanosec     │ 926902148                                  │ │
│ │ frame_id          │ imu_link                                   │ │
│ │ orient.x          │  0.578577                                  │ │
│ │ orient.y          │ -0.815619                                  │ │
│ │ orient.z          │ -0.003497                                  │ │
│ │ orient.w          │ -0.001703                                  │ │
│ │ lin_acc.z [m/s2]  │ -9.802000                                  │ │
│ │ ang_vel.x [rad/s] │ -0.005000                                  │ │
│ │ hz (aprox)        │ ~50 Hz                                     │ │
│ └───────────────────┴────────────────────────────────────────────┘ │
├─────────────────────────────────────────────────────────────────────┤
│ ● Conectado  /dev/ttyUSB0  115200  50Hz  — msg #1950               │
└─────────────────────────────────────────────────────────────────────┘
```

## 5. Package Structure

```
imu-dds/
└── ros2_ws/
    └── src/
        ├── imu_dds_adapter/              ← Driver package
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── config/
        │   │   ├── adapter_params.yaml   ← ROS 2 parameters
        │   │   └── imu_rviz.rviz         ← RViz2 config
        │   ├── launch/
        │   │   └── imu_dds.launch.py     ← ros2 launch (rviz:=true optional)
        │   ├── include/imu_dds_adapter/
        │   │   ├── imu_frame.hpp         ← DTO struct
        │   │   ├── nmea_parser.hpp       ← NMEA 0183 parser interface
        │   │   ├── imu_converter.hpp     ← NED→ENU converter interface
        │   │   ├── serial_port.hpp       ← UART interface
        │   │   └── imu_publisher_node.hpp← ROS 2 node interface
        │   ├── src/
        │   │   ├── nmea_parser.cpp
        │   │   ├── imu_converter.cpp
        │   │   ├── serial_port.cpp
        │   │   ├── imu_publisher_node.cpp
        │   │   └── main.cpp
        │   └── test/
        │       ├── test_nmea_parser.cpp  ← 10 GTest cases
        │       └── test_imu_converter.cpp← 6 GTest cases
        │
        └── imu_dds_monitor/              ← rqt plugin package
            ├── CMakeLists.txt
            ├── package.xml
            ├── plugin.xml                ← pluginlib descriptor
            ├── resource/
            │   └── imu_monitor.ui        ← Qt6 UI definition
            ├── include/imu_dds_monitor/
            │   └── imu_monitor_widget.hpp
            └── src/
                ├── imu_monitor_widget.cpp
                └── imu_monitor_plugin.cpp
```

## 6. DDS Topic Map

```
  Publisher                    Topic               Message Type              QoS
  ─────────────────────────────────────────────────────────────────────────────
  imu_dds_adapter_node  ──►  /imu/data           sensor_msgs/Imu           depth=10
  imu_dds_adapter_node  ──►  /imu/data_raw        sensor_msgs/Imu           depth=10
  imu_dds_adapter_node  ──►  /imu/raw_ascii       std_msgs/String           depth=10
  imu_dds_adapter_node  ──►  /imu/status          diagnostic_msgs/          depth=10
                                                   DiagnosticStatus

  Subscriber                   Topic               Message Type
  ─────────────────────────────────────────────────────────────────────────────
  ImuMonitorWidget      ◄──  /imu/data           sensor_msgs/Imu
  ImuMonitorWidget      ◄──  /imu/raw_ascii       std_msgs/String
  RViz2 IMU plugin      ◄──  /imu/data           sensor_msgs/Imu
  RViz2 TF             ◄──  /tf_static           tf2_msgs/TFMessage
```
