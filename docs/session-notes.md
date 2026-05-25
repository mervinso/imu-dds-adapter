# Notas de sesión — contexto para continuación

## Sesión 2026-05-24/25: DDS Spy + CycloneDDS Insight

### Estado del repo
- Branch: `master` — remoto: `https://github.com/mervinso/imu-dds-adapter`
- Último commit: `11b5cee` feat: add standalone DDS spy to verify IMU data outside ROS 2

### Herramientas instaladas en el sistema
| Herramienta | Ubicación | Notar |
|---|---|---|
| CycloneDDS Insight v11.0.1 | `/usr/bin/CycloneDDS Insight` | Lanzar con comillas por el espacio |
| Fast DDS Monitor v3.2.0 | Instalado vía .run | Requiere `FASTDDS_STATISTICS` env var |
| Fast DDS Spy v1.2.0 | Docker image cargada | `docker run` con network=host |
| imu_dds_spy | `tools/imu_dds_spy.cpp` | Compilar con `cmake -B build tools/` |

### Compilar imu_dds_spy
```bash
cd /home/cednav/adapter/imu-dds/tools
cmake -B build . && cmake --build build
# O con g++ directo:
g++ -std=c++17 imu_dds_spy.cpp \
  -I/opt/ros/lyrical/include/fastdds \
  -I/opt/ros/lyrical/include \
  -I/opt/ros/lyrical/includefastcdr \
  -L/opt/ros/lyrical/lib \
  -L/opt/ros/lyrical/lib/x86_64-linux-gnu \
  -lfastdds -lfastcdr \
  -Wl,-rpath,/opt/ros/lyrical/lib \
  -Wl,-rpath,/opt/ros/lyrical/lib/x86_64-linux-gnu \
  -o /tmp/imu_dds_spy
```

### Lanzar el adaptador
```bash
pkill -f imu_dds_adapter_node 2>/dev/null; sleep 1
source /opt/ros/lyrical/setup.bash
source /home/cednav/adapter/imu-dds/ros2_ws/install/setup.bash
ros2 launch imu_dds_adapter imu_dds.launch.py
```

### CycloneDDS Insight — limitación documentada
El Listener tab **no muestra payload** cross-vendor (Fast DDS → CycloneDDS) por TypeObject hash mismatch en XTypes TypeLookup Service. Es una limitación del estándar DDS-XTYPES en interoperabilidad cross-vendor.

**Qué SÍ funciona en Insight:**
- Descubrimiento de topics (Domain 0, `rt/imu/data` visible)
- QoS completo del publisher leído correctamente
- Topología Writer/Reader

**IDL con nombre ROS 2 mangled** (guardar en `/tmp/imu_ros2dds.idl` para importar en Insight):
```idl
module builtin_interfaces { module msg { module dds_ {
  struct Time_ { long sec; unsigned long nanosec; };
}}};
module std_msgs { module msg { module dds_ {
  struct Header_ { builtin_interfaces::msg::dds_::Time_ stamp; string frame_id; };
}}};
module sensor_msgs { module msg { module dds_ {
  struct Imu_ {
    std_msgs::msg::dds_::Header_ header;
    double orientation_x; double orientation_y;
    double orientation_z; double orientation_w;
    double orientation_covariance[9];
    double angular_velocity_x; double angular_velocity_y; double angular_velocity_z;
    double angular_velocity_covariance[9];
    double linear_acceleration_x; double linear_acceleration_y; double linear_acceleration_z;
    double linear_acceleration_covariance[9];
  };
}}};
```

### Claves Fast DDS 3.x (distinto de 2.x)
- Miembro clave: `is_compute_key_provided` (NO `m_isGetKeyDefined`)
- `take_next_sample(void* data, SampleInfo*)` — NO `SerializedPayload_t`
- Headers fastcdr: `/opt/ros/lyrical/includefastcdr/`
- Lib fastcdr: `/opt/ros/lyrical/lib/x86_64-linux-gnu/`

### Hallazgo técnico central (para la tesis)
```
Capa DDS            Estándar          Cross-vendor resultado
──────────────────  ────────────────  ─────────────────────
Transporte RTPS     Obligatorio       ✓ Siempre funciona
Discovery SPDP/SEDP Core              ✓ Topics visibles
CDR serialización   Obligatorio       ✓ Bytes idénticos
XTypes TypeLookup   Opcional          ✗ Divergente entre vendors
```

La interoperabilidad DDS es real a nivel de transporte y datos. La fricción aparece
solo en introspección de tipo en runtime (extensión opcional, no afecta transmisión).

### Pendiente
- [ ] Revisar Statistics tab en CycloneDDS Insight (throughput/latencia sin necesitar tipo)
- [ ] Documentar limitación TypeLookup en README o capítulo de tesis
