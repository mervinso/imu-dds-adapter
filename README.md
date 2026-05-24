# IMU-DDS Adapter

[![CI](https://github.com/mervinso/imu-dds-adapter/actions/workflows/ci.yml/badge.svg)](https://github.com/mervinso/imu-dds-adapter/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Lyrical-blue?logo=ros)](https://docs.ros.org)
[![Release](https://img.shields.io/github/v/release/mervinso/imu-dds-adapter)](https://github.com/mervinso/imu-dds-adapter/releases)

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
        │   ├── meshes/
        │   │   └── vn100_rugged.stl      # Modelo 3D real del sensor (STEP→STL)
        │   ├── urdf/
        │   │   └── vn100.urdf            # Descripción del robot para visualización
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
  ros-lyrical-pluginlib \
  ros-lyrical-robot-state-publisher \
  ros-lyrical-rviz-imu-plugin \
  ros-lyrical-foxglove-bridge
```

### Acceso al puerto serie y symlink fijo
```bash
sudo usermod -aG dialout $USER
# Cerrar sesión y volver a entrar para que tome efecto

# Regla udev para asignar /dev/imu fijo al sensor (Moxa UPort 1150)
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="110a", ATTRS{idProduct}=="1150", SYMLINK+="imu", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-vn100-imu.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Después de reconectar el sensor estará siempre disponible en `/dev/imu` sin importar el puerto USB.

### Hardware
- Sensor VectorNav VN-100S-CR conectado por USB-UART (adaptador Moxa UPort 1150)
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

### Foxglove Studio (visualización avanzada)

Foxglove Studio permite suscribirse a cualquier tópico DDS en tiempo real desde el navegador, demostrando el desacoplamiento publisher/subscriber de DDS.

```bash
# Terminal 1 — adaptador (incluye robot_state_publisher con el modelo 3D)
ros2 launch imu_dds_adapter imu_dds.launch.py

# Terminal 2 — bridge WebSocket
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Abrir Foxglove Studio en el navegador y conectar a `ws://localhost:8765`.

**Paneles recomendados:**

| Panel | Configuración |
|---|---|
| 3D | Fixed frame: `map` — muestra el modelo VN-100 rotando en tiempo real |
| Plot | Series: `/imu/data.linear_acceleration.x/y/z` |
| Raw Messages | Topic: `/imu/data` — inspección campo a campo |
| Diagnostics | Topic: `/imu/status` — frames recibidos y errores |

---

## Parámetros de configuración

Archivo: `ros2_ws/src/imu_dds_adapter/config/adapter_params.yaml`

| Parámetro | Valor por defecto | Descripción |
|---|---|---|
| `serial_port` | `/dev/imu` | Puerto serie del sensor (symlink udev fijo) |
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
| `/tf` | `tf2_msgs/TFMessage` | Transform dinámico `map → imu_link` a 50 Hz con la orientación real del sensor |
| `/robot_description` | `std_msgs/String` | URDF del VN-100 para visualización 3D en RViz y Foxglove |

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

## Modelo 3D del sensor

El paquete incluye el modelo CAD real del VN-100 Rugged convertido de STEP a STL:

```
ros2_ws/src/imu_dds_adapter/
├── meshes/vn100_rugged.stl   # Malla 3D (STEP original → gmsh → STL, escala 1 mm = 0.001 m)
└── urdf/vn100.urdf            # URDF que referencia el mesh en imu_link
```

El nodo `robot_state_publisher` se lanza automáticamente con el adaptador y publica `/robot_description`. Tanto RViz2 (con `rviz_imu_plugin`) como Foxglove Studio (panel 3D) renderizan el modelo rotando en tiempo real usando el TF dinámico `map → imu_link`.

Para regenerar el STL desde el archivo STEP original:
```bash
sudo apt install gmsh
gmsh vn100_rugged.step -3 -format stl -o vn100_rugged.stl -v 0
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

## Verificación manual en Lyrical

El badge de CI documenta los pasos pero no ejecuta el build en los runners de GitHub, ya que ROS 2 Lyrical requiere instalación local (no está disponible como imagen de GitHub Actions). Para verificar el proyecto en tu máquina con Lyrical instalado:

### 1. Build completo

```bash
cd imu-dds-adapter/ros2_ws
source /opt/ros/lyrical/setup.bash

colcon build \
  --build-base build \
  --install-base install \
  2>&1 | grep -E "Starting|Finished|Failed"
```

Salida esperada:
```
Starting >>> imu_dds_adapter
Starting >>> imu_dds_monitor
Finished <<< imu_dds_adapter [Xs]
Finished <<< imu_dds_monitor [Xs]
```

### 2. Tests unitarios

```bash
source /opt/ros/lyrical/setup.bash

colcon test \
  --build-base build \
  --install-base install \
  --packages-select imu_dds_adapter

colcon test-result --verbose
```

Salida esperada (16 tests):
```
build/imu_dds_adapter/test_results/.../test_nmea_parser.xml:
  10 tests, 0 errors, 0 failures

build/imu_dds_adapter/test_results/.../test_imu_converter.xml:
  6 tests, 0 errors, 0 failures
```

### 3. Verificación con sensor real

Con el VN-100S-CR conectado (disponible en `/dev/imu` tras instalar la regla udev):

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash

# Terminal 1 — arrancar el adaptador (una sola instancia)
ros2 launch imu_dds_adapter imu_dds.launch.py

# Terminal 2 — verificar publicación
ros2 topic hz /imu/data           # debe mostrar ~50 Hz
ros2 topic echo /imu/data --once  # verificar linear_acceleration.z ≈ -9.8

# Terminal 2 — ver diagnósticos
ros2 topic echo /imu/status --once
# checksum_errors debe ser 0
```

### 4. Plugin rqt

```bash
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash

rqt --standalone imu_dds_monitor/ImuMonitorPlugin
```

---

## Solución de problemas

**El nodo no abre `/dev/imu`**
```bash
ls -l /dev/imu               # Verificar que el symlink existe
ls -l /dev/ttyUSB*           # Ver qué puerto asignó el kernel
groups $USER | grep dialout  # Verificar permisos de grupo
```
Si `/dev/imu` no existe, reconectar el USB o recargar las reglas udev:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Datos corruptos / checksum errors**
```bash
# Causa más común: múltiples instancias del adaptador leyendo el mismo puerto
pgrep -c -f imu_dds_adapter_node   # debe decir 1
pkill -f imu_dds_adapter_node       # matar todas y relanzar una sola
```

**El modelo 3D no aparece en Foxglove**
```bash
# Verificar que robot_state_publisher está corriendo
ros2 topic echo /robot_description --once | head -5
```
En el panel 3D de Foxglove, sección **Topics**, activar `/robot_description`.

**El plugin rqt no aparece en la lista**
```bash
ros2 run rqt_gui rqt_gui --list-plugins | grep imu
# Debe mostrar: imu_dds_monitor/ImuMonitorPlugin
```
Si no aparece, asegúrate de haber hecho `source ros2_ws/install/setup.bash` antes de lanzar rqt.
