# IMU-DDS Adapter

Adaptador ROS 2 entre el protocolo NMEA 0183 del sensor inercial **VectorNav VN-100S-CR** y el estándar de middleware **OMG DDS**, desarrollado como proyecto de tesis de maestría.

El sistema lee datos IMU en ASCII desde el sensor por UART a 50 Hz, los parsea, aplica la rotación de referencia NED→ENU y los publica como mensajes `sensor_msgs/Imu` sobre Fast-DDS. Incluye un plugin de monitorización para rqt con tres vistas en tiempo real.

---

## Stack tecnológico

| Componente | Versión |
|---|---|
| ROS 2 | Lyrical (rolling, 2026) |
| Middleware DDS | Fast-DDS (RMW por defecto) |
| Compilador | GCC 14 / C++17 |
| CMake | 4.2 |
| Qt | 6.x (plugin rqt) |
| Sensor | VectorNav VN-100S-CR |
| Protocolo | NMEA 0183 — registro `$VNIMU` (Reg 54) + polling `$VNRRG,09` |

---

## Arquitectura

```
VN-100S-CR ──UART 115200──► SerialPort
                                │
                    ┌───────────┴──────────┐
                    │ Thread A (readline)   │ Thread B (ROS timer 50 Hz)
                    └───────────┬──────────┘
                                │ frame_queue_  [mutex]
                                ▼
                          NmeaParser::parseVnimu()
                                │
                          ImuFrame (DTO)
                                │
                          ImuConverter::convert()   ← NED→ENU + covarianzas
                                │
                     ┌──────────┼──────────┐
                     ▼          ▼          ▼
               /imu/data   /imu/data_raw  /imu/raw_ascii
               /imu/status
                     │
              Fast-DDS / ROS 2
                     │
          ┌──────────┴──────────┐
          ▼                     ▼
    imu_dds_monitor          RViz2
    (rqt plugin)         (rviz_imu_plugin)
```

**Dos paquetes ROS 2:**
- `imu_dds_adapter` — nodo driver: lectura UART, parsing, conversión, publicación DDS
- `imu_dds_monitor` — plugin rqt: visualización Raw ASCII, Parsed Fields, DDS Output

---

## Estructura del repositorio

```
imu-dds-adapter/
├── docs/
│   └── diagrams/
│       └── architecture.md       # Diagramas ASCII de arquitectura y flujo de datos
└── ros2_ws/
    └── src/
        ├── imu_dds_adapter/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── config/
        │   │   ├── adapter_params.yaml   # Parámetros ROS 2
        │   │   └── imu_rviz.rviz         # Configuración RViz2
        │   ├── launch/
        │   │   └── imu_dds.launch.py
        │   ├── include/imu_dds_adapter/
        │   │   ├── imu_frame.hpp         # DTO de datos IMU
        │   │   ├── nmea_parser.hpp       # Parser NMEA 0183
        │   │   ├── imu_converter.hpp     # Conversor NED→ENU
        │   │   ├── serial_port.hpp       # Interfaz UART
        │   │   └── imu_publisher_node.hpp
        │   ├── src/
        │   │   ├── nmea_parser.cpp
        │   │   ├── imu_converter.cpp
        │   │   ├── serial_port.cpp
        │   │   ├── imu_publisher_node.cpp
        │   │   └── main.cpp
        │   └── test/
        │       ├── test_nmea_parser.cpp  # 10 casos GTest
        │       └── test_imu_converter.cpp # 6 casos GTest
        └── imu_dds_monitor/
            ├── CMakeLists.txt
            ├── package.xml
            ├── plugin.xml
            ├── resource/
            │   └── imu_monitor.ui
            ├── include/imu_dds_monitor/
            │   └── imu_monitor_widget.hpp
            └── src/
                ├── imu_monitor_widget.cpp
                └── imu_monitor_plugin.cpp
```

---

## Prerequisitos

### Sistema operativo
Ubuntu 24.04 LTS (Noble) o superior.

### ROS 2
```bash
# Instalar ROS 2 Lyrical
sudo apt install ros-lyrical-desktop

# Dependencias del adaptador
sudo apt install \
  ros-lyrical-rclcpp \
  ros-lyrical-sensor-msgs \
  ros-lyrical-std-msgs \
  ros-lyrical-diagnostic-msgs \
  ros-lyrical-geometry-msgs \
  ros-lyrical-tf2 \
  ros-lyrical-tf2-ros \
  ros-lyrical-rqt-gui-cpp \
  ros-lyrical-qt-gui-cpp \
  ros-lyrical-pluginlib
```

### Acceso al puerto serie
```bash
sudo usermod -aG dialout $USER
# Cerrar sesión y volver a entrar para que tome efecto
```

### Hardware
- Sensor VectorNav VN-100S-CR conectado por USB-UART (`/dev/ttyUSB0`)
- El sensor debe estar en modo ASCII (configuración de fábrica)

---

## Instalación

```bash
git clone https://github.com/mervinso/imu-dds-adapter.git
cd imu-dds-adapter/ros2_ws

source /opt/ros/lyrical/setup.bash

colcon build \
  --build-base build \
  --install-base install

source install/setup.bash
```

### Compilar y ejecutar tests unitarios
```bash
colcon test \
  --build-base build \
  --install-base install \
  --packages-select imu_dds_adapter

colcon test-result --verbose
```

Los 16 tests (10 NmeaParser + 6 ImuConverter) deben pasar sin errores.

---

## Ejecución

### Adaptador solo
```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash

ros2 launch imu_dds_adapter imu_dds.launch.py
```

### Adaptador + RViz2
```bash
ros2 launch imu_dds_adapter imu_dds.launch.py rviz:=true
```

### Plugin de monitorización rqt (terminal separada)
```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash

rqt --standalone imu_dds_monitor/ImuMonitorPlugin
```

---

## Parámetros de configuración

Archivo: `ros2_ws/src/imu_dds_adapter/config/adapter_params.yaml`

| Parámetro | Valor por defecto | Descripción |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | Puerto serie del sensor |
| `baud_rate` | `115200` | Velocidad UART |
| `output_hz` | `50` | Frecuencia de publicación |
| `quat_poll_every_n` | `5` | Polling de cuaternión cada N frames |
| `frame_id` | `imu_link` | Frame TF del mensaje IMU |
| `reconfigure_sensor` | `true` | Enviar comandos de configuración al arrancar |

Para cambiar parámetros en tiempo de lanzamiento:
```bash
ros2 launch imu_dds_adapter imu_dds.launch.py \
  serial_port:=/dev/ttyUSB1 \
  output_hz:=100
```

---

## Tópicos DDS publicados

| Tópico | Tipo | Descripción |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | IMU completo con orientación validada (ENU) |
| `/imu/data_raw` | `sensor_msgs/Imu` | IMU sin filtrar — siempre publicado |
| `/imu/raw_ascii` | `std_msgs/String` | Línea NMEA raw del sensor |
| `/imu/status` | `diagnostic_msgs/DiagnosticStatus` | Frames recibidos, descartados y errores de checksum |

### Verificar publicación
```bash
ros2 topic hz /imu/data          # Debe mostrar ~50 Hz
ros2 topic echo /imu/data --once # Ver un mensaje completo
```

---

## Conversión de referencia NED→ENU

El VN-100 publica cuaterniones en el sistema de coordenadas NED (North-East-Down). ROS 2 usa ENU (East-North-Up) según REP-103. La conversión se realiza multiplicando por:

```
q_rot = 180° alrededor del eje (1/√2, 1/√2, 0)
q_enu = q_rot ⊗ q_ned
```

---

## Covarianzas

Las matrices de covarianza del mensaje `sensor_msgs/Imu` están fijadas a los valores de la hoja de datos del VN-100:

| Magnitud | Varianza diagonal |
|---|---|
| Orientación | 1×10⁻⁴ rad² |
| Velocidad angular | 1.2×10⁻⁷ rad²/s² |
| Aceleración lineal | 4×10⁻⁵ m²/s⁴ |

Cuando el cuaternión no ha sido polleado aún, `orientation_covariance[0] = -1` según REP-145.

---

## Solución de problemas

**El nodo no abre `/dev/ttyUSB0`**
```bash
ls -l /dev/ttyUSB0          # Verificar que existe
groups $USER | grep dialout  # Verificar permisos
```

**Datos corruptos / checksum errors**
```bash
# Verificar que no hay otros procesos usando el puerto
fuser /dev/ttyUSB0
```

**El plugin rqt no aparece en la lista**
```bash
ros2 run rqt_gui rqt_gui --list-plugins | grep imu
# Debe mostrar: imu_dds_monitor/ImuMonitorPlugin
```
Si no aparece, asegúrate de haber hecho `source ros2_ws/install/setup.bash` antes de lanzar rqt.
