# IMU-DDS Adapter — Especificación de Diseño

**Fecha:** 2026-05-23
**Versión:** 1.0
**Distribución:** ROS 2 Lyrical
**Sensor:** VectorNav VN-100S-CR (Firmware 3.0.0.0)
**Interfaz física:** Moxa UPort 1150 → /dev/ttyUSB0

---

## 1. Propósito

Diseñar e implementar un adaptador de software que recibe datos NMEA 0183 (ASCII) del sensor inercial VN-100S-CR por puerto serial, los convierte al estándar DDS (OMG Data Distribution Service), y los publica dentro del ecosistema ROS 2 Lyrical para su visualización en RViz2.

El artefacto implementa el patrón **Adapter** (GoF) sobre el patrón **Gateway** (OMG OPC UA/DDS Gateway v1.0) como referencia arquitectónica formal.

---

## 2. Contexto del Sistema

### Sensor

| Parámetro | Valor |
|---|---|
| Modelo | VN-100S-CR |
| Firmware | 3.0.0.0 |
| Puerto | /dev/ttyUSB0 |
| Baud rate | 115200 |
| Protocolo | ASCII asíncrono (NMEA-like) |
| Frecuencia | 50 Hz |
| Checksum | XOR 8-bit |
| Output actual | $VNYPR (solo Yaw/Pitch/Roll) |
| Output requerido | Registro 54 ($VNIMU) + polling Registro 9 (cuaternión) |

### Decisiones de diseño

| Decisión | Elección | Justificación |
|---|---|---|
| Protocolo VN-100 | ASCII a 50 Hz | Suficiente para tesis; parser más simple y legible |
| Datos publicados | IMU completo (accel + gyro + orientación) | Cumple sensor_msgs/Imu estándar para RViz2 |
| Arquitectura | Pipeline en dos paquetes | Balance entre rigor académico y complejidad |
| Interfaz | Plugin rqt | Integrado al ecosistema ROS 2 |
| DDS implementation | Fast-DDS (rmw por defecto en ROS 2 Lyrical) | Estándar industrial, open source, justificable ante comité |

---

## 3. Arquitectura General

### Dos paquetes en un workspace ROS 2

```
ros2_ws/
└── src/
    ├── imu_dds_adapter/     ← nodo principal: pipeline serial → DDS
    └── imu_dds_monitor/     ← plugin rqt: suscriptor + visualización
```

### Vista de capas

```
┌─────────────────────────────────────────────────────────────┐
│                    imu_dds_adapter                          │
│                                                             │
│  ┌──────────────┐   ┌──────────────┐   ┌────────────────┐  │
│  │  Serial      │   │  NMEA        │   │  ROS 2         │  │
│  │  Driver      │──▶│  Parser      │──▶│  Publisher     │  │
│  │  Layer       │   │  Layer       │   │  Layer (DDS)   │  │
│  └──────────────┘   └──────────────┘   └────────────────┘  │
│        ▲                   │                    │           │
│        │            ┌──────▼──────┐             │           │
│        │            │  Converter  │             │           │
│   /dev/ttyUSB0      │  Layer      │             │           │
│                     └─────────────┘             │           │
└─────────────────────────────────────────────────┼───────────┘
                                                  │ DDS/RTPS
┌─────────────────────────────────────────────────▼───────────┐
│                    imu_dds_monitor                           │
│              plugin rqt — suscriptor DDS                    │
│         [Raw ASCII] [Datos parseados] [Campos DDS]          │
└─────────────────────────────────────────────────────────────┘
                                                  │
                                            ┌─────▼──────┐
                                            │   RViz2    │
                                            │  /imu/data │
                                            └────────────┘
```

### Responsabilidades por capa

| Capa | Responsabilidad | Patrón |
|---|---|---|
| Serial Driver | Abrir puerto, configurar baud rate, reconfigurar sensor al inicio, leer bytes | Hardware Abstraction |
| NMEA Parser | Tokenizar mensajes ASCII $VN..., validar checksum XOR, extraer campos | Chain of Responsibility |
| Converter | Mapear campos VN-100 → sensor_msgs/Imu (unidades, marcos de referencia) | Adapter (GoF) |
| ROS 2 Publisher | Crear DomainParticipant, Topic, DataWriter con QoS; publicar a 50 Hz | Publisher (DDS DCPS) |
| rqt Monitor | Suscribir a topics DDS, mostrar 3 vistas en tiempo real | Observer |

---

## 4. Pipeline Interno

### Modelo de threads

```
┌─────────────────────────────────────────────────────────────────┐
│  Thread A — Serial Reader                                        │
│                                                                  │
│  loop:                                                           │
│    line = serial.readline()          ← bloqueante                │
│    if validate_checksum(line): OK                                │
│      queue.push(line)                ← no parsea, solo encola   │
│    else: log warning, skip                                       │
└───────────────────────────┬─────────────────────────────────────┘
                            │  std::queue<std::string>
                            │  protegida con std::mutex
                            │  + std::condition_variable
┌───────────────────────────▼─────────────────────────────────────┐
│  Thread B — ROS 2 Executor (spin)                                │
│                                                                  │
│  Timer callback @ 50 Hz:                                         │
│    if queue.empty(): return          ← sin bloqueo               │
│    line = queue.pop()                                            │
│    fields = NmeaParser::parse(line)                              │
│    imu_msg = ImuConverter::convert(fields)                       │
│    publisher->publish(imu_msg)       ← DDS DataWriter            │
└─────────────────────────────────────────────────────────────────┘
```

### Componentes internos

**`SerialPort`**
```
open(device, baud_rate)
configure_sensor()    ← VNASY,0 → VNWRG,6,19 → VNWRG,7,50 → VNASY,1
readline() → string
close()
```

**`NmeaParser`** — sin estado
```
parse(raw_line) → ImuFrame
  • valida '$' inicial y checksum XOR post-'*'
  • tokeniza por comas
  • retorna ImuFrame con campos tipados
```

**`ImuFrame`** — estructura interna de transferencia
```cpp
struct ImuFrame {
    double mag_x, mag_y, mag_z;        // Gauss
    double accel_x, accel_y, accel_z;  // m/s²
    double gyro_x, gyro_y, gyro_z;     // rad/s
    double temperature;                 // °C
    double pressure;                    // kPa
    double qx, qy, qz, qw;            // cuaternión (del polling Reg 9)
    uint64_t timestamp_ns;             // clock del nodo ROS 2
};
```

**`ImuConverter`**
```
convert(ImuFrame) → sensor_msgs::msg::Imu
  • linear_acceleration: accel_x/y/z (m/s², sin conversión)
  • angular_velocity:    gyro_x/y/z  (rad/s, sin conversión)
  • orientation:         qx,qy,qz,qw
  • covariance matrices: diagonal con varianza del VN-100S-CR
  • Rotación NED → ENU:  x_ENU=y_NED, y_ENU=x_NED, z_ENU=-z_NED
  • header.frame_id: "imu_link"
  • header.stamp: rclcpp::Clock
```

### Manejo del cuaternión (polling complementario)

El Registro 54 (stream principal) entrega Mag+Accel+Gyro+Temp+Presión sin orientación. El cuaternión se obtiene por polling del Registro 9:

```
Cada N ciclos del Timer (configurable, default N=5 → cada 100ms):
  Thread B envía $VNRRG,09*XX por el mismo puerto serial
  Thread A recibe $VNRRG,09,qx,qy,qz,qw*CS
  NmeaParser detecta tipo VNRRG,09 y actualiza quat_cache_
  ImuConverter usa quat_cache_ en la siguiente publicación
```

`quat_cache_` protegida con mutex liviano. Actualización a ~10 Hz, suficiente para RViz2.

### Flujo de inicio

```
1. Inicializar nodo ROS 2
2. Abrir /dev/ttyUSB0 a 115200 baud
3. $VNASY,0    → pausar stream asíncrono
4. $VNWRG,6,19 → configurar output a Registro 54 (VNIMU)
5. $VNWRG,7,50 → confirmar 50 Hz
6. $VNASY,1    → reanudar stream
7. Arrancar Thread A (Serial Reader)
8. Arrancar Thread B (ROS 2 Executor con Timer @ 50 Hz)
9. Publicar en /imu/data, /imu/data_raw, /imu/status
```

---

## 5. Topics DDS

### Topics publicados

| Topic | Tipo ROS 2 | Fuente | Descripción |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | Reg 54 + Reg 9 poll | Datos compensados — entrada principal de RViz2 |
| `/imu/data_raw` | `sensor_msgs/Imu` | Reg 54 (UncompAccel/Gyro) | Datos sin compensar — diagnóstico |
| `/imu/status` | `diagnostic_msgs/DiagnosticStatus` | AhrsStatus bitfield | Calidad de actitud, saturación, métricas |

### Anatomía de `/imu/data`

```
sensor_msgs/Imu:
  header:
    stamp:    rclcpp::Clock
    frame_id: "imu_link"

  orientation:              qx, qy, qz, qw    ← Reg 9 (poll ~10 Hz)
  orientation_covariance:   [0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001]

  angular_velocity:         gyro_x, gyro_y, gyro_z   ← Reg 54 (50 Hz)
  angular_velocity_covariance:   [1.2e-7, 0, 0, 0, 1.2e-7, 0, 0, 0, 1.2e-7]

  linear_acceleration:      accel_x, accel_y, accel_z ← Reg 54 (50 Hz)
  linear_acceleration_covariance: [4e-5, 0, 0, 0, 4e-5, 0, 0, 0, 4e-5]
```

Los valores de covarianza se derivan del datasheet VN-100S-CR:
- Ruido giroscopio: 0.0035°/s/√Hz → 1.2×10⁻⁷ rad²/s²
- Ruido acelerómetro: 0.0028 m/s²/√Hz → 4×10⁻⁵ m²/s⁴

### Marco de referencia

El VN-100 usa NED (North-East-Down). ROS 2 usa ENU (East-North-Up).
El `ImuConverter` aplica la rotación estática NED → ENU.
Se publica adicionalmente un transform tf2 estático `map → imu_link`.

### QoS del DataWriter DDS

```xml
<reliability>  BEST_EFFORT     </reliability>
<durability>   VOLATILE        </durability>
<history>      KEEP_LAST 1     </history>
<deadline>     20ms            </deadline>
<liveliness>   AUTOMATIC 500ms </liveliness>
```

---

## 6. Estructura de Paquetes

```
ros2_ws/
└── src/
    ├── imu_dds_adapter/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/imu_dds_adapter/
    │   │   ├── serial_port.hpp
    │   │   ├── nmea_parser.hpp
    │   │   ├── imu_frame.hpp
    │   │   ├── imu_converter.hpp
    │   │   └── imu_publisher_node.hpp
    │   ├── src/
    │   │   ├── serial_port.cpp
    │   │   ├── nmea_parser.cpp
    │   │   ├── imu_converter.cpp
    │   │   ├── imu_publisher_node.cpp
    │   │   └── main.cpp
    │   ├── config/
    │   │   └── adapter_params.yaml
    │   └── launch/
    │       └── imu_dds.launch.py
    │
    └── imu_dds_monitor/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include/imu_dds_monitor/
        │   └── imu_monitor_widget.hpp
        ├── src/
        │   ├── imu_monitor_plugin.cpp
        │   └── imu_monitor_widget.cpp
        ├── resource/
        │   └── imu_monitor.ui
        └── plugin.xml
```

### Dependencias `imu_dds_adapter`

```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>diagnostic_msgs</depend>
<depend>tf2_ros</depend>
<depend>geometry_msgs</depend>
```

### Dependencias `imu_dds_monitor`

```xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>rqt_gui</depend>
<depend>rqt_gui_cpp</depend>
<depend>qt_gui_cpp</depend>
```

### Archivo de parámetros (`adapter_params.yaml`)

```yaml
imu_dds_adapter:
  ros__parameters:
    serial_port:        /dev/ttyUSB0
    baud_rate:          115200
    output_hz:          50
    quat_poll_every_n:  5
    frame_id:           imu_link
    reconfigure_sensor: true
```

---

## 7. Plugin rqt

### Tres paneles en tabs

```
┌─────────────────────────────────────────────────────────┐
│  IMU-DDS Monitor                              [rqt]     │
├──────────────┬──────────────────┬───────────────────────┤
│  Raw ASCII   │  Parsed Fields   │  DDS Output           │
├──────────────┼──────────────────┼───────────────────────┤
│ $VNIMU,      │ accel_x: -0.078  │ topic: /imu/data      │
│ -00.1462,    │ accel_y: +0.070  │ seq:   1842           │
│ +00.1761,    │ accel_z: -9.801  │ stamp: 1748041...     │
│ +00.2623,    │ gyro_x:  +0.0079 │ frame: imu_link       │
│ -00.078,     │ gyro_y:  -0.0034 │ orient.x:  0.004846   │
│ +00.070,     │ gyro_z:  -0.0093 │ orient.y:  0.000221   │
│ -09.801,     │ quat_x:  0.0048  │ orient.z:  0.905755   │
│ +00.0079,... │ quat_y:  0.0002  │ orient.w: -0.423774   │
│              │ temp:   36.3 °C  │ lin_acc.z: -9.801     │
│              │ press: 100.87kPa │ ang_vel.x:  0.007910  │
│              │                  │ status: EXCELLENT      │
│  [scroll]    │                  │ hz: 49.8 Hz           │
├──────────────┴──────────────────┴───────────────────────┤
│  ● Conectado  /dev/ttyUSB0  115200  50Hz   [Desconectar]│
└─────────────────────────────────────────────────────────┘
```

### Estructura interna

```
ImuMonitorPlugin        ← rqt_gui_cpp::Plugin
  └── ImuMonitorWidget  ← QWidget (layout desde .ui)
        ├── QTextEdit       raw_panel_
        ├── QTableWidget    parsed_panel_
        ├── QTableWidget    dds_panel_
        ├── QLabel          status_bar_
        └── rclcpp::Subscription<sensor_msgs::msg::Imu>
              └── callback → buffer → QTimer @ 10 Hz → actualiza UI
```

El callback ROS 2 no toca la UI directamente. Un `QTimer` a 10 Hz refresca los paneles desde el hilo Qt (thread safety Qt).

---

## 8. Manejo de Errores

### Errores de comunicación serial

| Condición | Comportamiento |
|---|---|
| Puerto no disponible al inicio | Log fatal, nodo termina con código de error |
| Checksum XOR inválido | Log warning, frame descartado, contador en /imu/status |
| Timeout >500ms sin datos | Log error, reconfiguración del sensor, máx. 3 reintentos |
| Error VNERR del sensor | Log error con código VN-100, frame descartado |
| Desconexión física | Log fatal, liveliness_lost en DDS, shutdown limpio |

### Errores de parsing

| Condición | Comportamiento |
|---|---|
| Mensaje incompleto | Frame descartado, sin interrupción del stream |
| Campo no numérico | Frame descartado, log warning con raw line |
| Mensaje desconocido | Ignorado por filtro de header |

### Errores DDS

| Condición | Comportamiento |
|---|---|
| Sin suscriptores | Normal — BEST_EFFORT no requiere suscriptores |
| Cola interna llena | Descarta frame más antiguo — dato IMU obsoleto sin valor |

### Estrategia de reconexión

```
on_timeout():
  retries = 0
  while retries < 3:
    send($VNASY,0)
    send($VNWRG,6,19)
    send($VNASY,1)
    wait(1s)
    if data_received: break
    retries++
  if retries == 3:
    RCLCPP_FATAL → shutdown()
```

### Diagnóstico en `/imu/status` (1 Hz)

```
level:   OK / WARN / ERROR
name:    "VN-100 IMU Adapter"
message: "Streaming 49.8 Hz"
values:
  checksum_errors:  0
  frames_received:  9842
  frames_dropped:   0
  quat_poll_hz:     9.9
  sensor_temp:      36.3
  ahrs_quality:     EXCELLENT
```

---

## 9. Criterios de Éxito

| Criterio | Verificación |
|---|---|
| El adaptador publica `/imu/data` a 50 Hz | `ros2 topic hz /imu/data` |
| RViz2 muestra la orientación IMU en tiempo real | Visual en RViz2 con plugin imu |
| El plugin rqt muestra los tres paneles con datos en vivo | Inspección visual |
| Checksum XOR validado en cada frame | Log sin errores de checksum |
| El sensor se reconfigura automáticamente al inicio | Log de secuencia VNWRG |
| El nodo termina limpiamente ante desconexión | Test de desconexión USB |
