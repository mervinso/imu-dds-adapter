# IMU-DDS Adapter — Plan de Implementación

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implementar un adaptador ROS 2 que lee datos NMEA 0183 ASCII del sensor VN-100S-CR por /dev/ttyUSB0 y los publica como sensor_msgs/Imu en DDS a 50 Hz, con visualización en RViz2 y plugin rqt de monitoreo.

**Architecture:** Pipeline de dos threads (Serial Reader + ROS 2 Executor) dentro de un nodo C++ (`imu_dds_adapter`), más un plugin rqt separado (`imu_dds_monitor`). El adaptador reconfigura el sensor al inicio para transmitir Registro 54 ($VNIMU) y hace polling del cuaternión (Registro 9) cada 100 ms.

**Tech Stack:** C++17, ROS 2 Lyrical, Fast-DDS (rmw por defecto), sensor_msgs/Imu, tf2, Qt5, rqt_gui_cpp, termios (UART), gtest.

---

## Mapa de Archivos

```
ros2_ws/src/
├── imu_dds_adapter/
│   ├── CMakeLists.txt                          CREAR
│   ├── package.xml                             CREAR
│   ├── include/imu_dds_adapter/
│   │   ├── imu_frame.hpp                       CREAR  — struct ImuFrame (DTO)
│   │   ├── nmea_parser.hpp                     CREAR  — validateChecksum, parseVnimu, parseQuaternion
│   │   ├── imu_converter.hpp                   CREAR  — convert(ImuFrame) → sensor_msgs/Imu
│   │   ├── serial_port.hpp                     CREAR  — open, configureSensor, readline, write, close
│   │   └── imu_publisher_node.hpp              CREAR  — rclcpp::Node con threading
│   ├── src/
│   │   ├── nmea_parser.cpp                     CREAR
│   │   ├── imu_converter.cpp                   CREAR
│   │   ├── serial_port.cpp                     CREAR
│   │   ├── imu_publisher_node.cpp              CREAR
│   │   └── main.cpp                            CREAR
│   ├── test/
│   │   ├── test_nmea_parser.cpp                CREAR
│   │   └── test_imu_converter.cpp              CREAR
│   ├── config/
│   │   └── adapter_params.yaml                 CREAR
│   └── launch/
│       └── imu_dds.launch.py                   CREAR
│
└── imu_dds_monitor/
    ├── CMakeLists.txt                          CREAR
    ├── package.xml                             CREAR
    ├── plugin.xml                              CREAR
    ├── include/imu_dds_monitor/
    │   └── imu_monitor_widget.hpp              CREAR
    ├── src/
    │   ├── imu_monitor_plugin.cpp              CREAR
    │   └── imu_monitor_widget.cpp              CREAR
    └── resource/
        └── imu_monitor.ui                      CREAR
```

---

## Task 1: Workspace y Scaffolding de Paquetes

**Files:**
- Create: `~/ros2_ws/src/imu_dds_adapter/CMakeLists.txt`
- Create: `~/ros2_ws/src/imu_dds_adapter/package.xml`
- Create: `~/ros2_ws/src/imu_dds_monitor/CMakeLists.txt`
- Create: `~/ros2_ws/src/imu_dds_monitor/package.xml`

- [ ] **Step 1: Crear workspace y directorios**

```bash
source /opt/ros/lyrical/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
mkdir -p imu_dds_adapter/include/imu_dds_adapter
mkdir -p imu_dds_adapter/src
mkdir -p imu_dds_adapter/test
mkdir -p imu_dds_adapter/config
mkdir -p imu_dds_adapter/launch
mkdir -p imu_dds_monitor/include/imu_dds_monitor
mkdir -p imu_dds_monitor/src
mkdir -p imu_dds_monitor/resource
```

- [ ] **Step 2: Crear `imu_dds_adapter/package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>imu_dds_adapter</name>
  <version>1.0.0</version>
  <description>Adaptador NMEA 0183 (VN-100) a DDS sobre ROS 2</description>
  <maintainer email="sosam@utb.edu.co">mervinso</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>diagnostic_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: Crear `imu_dds_adapter/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(imu_dds_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(ADAPTER_SOURCES
  src/nmea_parser.cpp
  src/imu_converter.cpp
  src/serial_port.cpp
  src/imu_publisher_node.cpp
)

add_executable(imu_dds_adapter_node src/main.cpp ${ADAPTER_SOURCES})

target_include_directories(imu_dds_adapter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(imu_dds_adapter_node
  rclcpp sensor_msgs std_msgs diagnostic_msgs geometry_msgs tf2 tf2_ros
)

install(TARGETS imu_dds_adapter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_nmea_parser
    test/test_nmea_parser.cpp
    src/nmea_parser.cpp
  )
  target_include_directories(test_nmea_parser PRIVATE include)

  ament_add_gtest(test_imu_converter
    test/test_imu_converter.cpp
    src/imu_converter.cpp
  )
  target_include_directories(test_imu_converter PRIVATE include)
  ament_target_dependencies(test_imu_converter sensor_msgs geometry_msgs tf2)
endif()

ament_package()
```

- [ ] **Step 4: Crear `imu_dds_monitor/package.xml`**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>imu_dds_monitor</name>
  <version>1.0.0</version>
  <description>Plugin rqt para monitoreo del adaptador IMU-DDS</description>
  <maintainer email="sosam@utb.edu.co">mervinso</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>rqt_gui</depend>
  <depend>rqt_gui_cpp</depend>
  <depend>qt_gui_cpp</depend>
  <depend>pluginlib</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 5: Crear `imu_dds_monitor/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(imu_dds_monitor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rqt_gui REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(pluginlib REQUIRED)

# Intentar Qt6 primero (Lyrical), luego Qt5
find_package(Qt6 QUIET COMPONENTS Core Gui Widgets)
if(NOT Qt6_FOUND)
  find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
  set(QT_LIBRARIES Qt5::Core Qt5::Gui Qt5::Widgets)
else()
  set(QT_LIBRARIES Qt6::Core Qt6::Gui Qt6::Widgets)
endif()

add_library(imu_dds_monitor SHARED
  src/imu_monitor_plugin.cpp
  src/imu_monitor_widget.cpp
)

target_include_directories(imu_dds_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(imu_dds_monitor ${QT_LIBRARIES})

ament_target_dependencies(imu_dds_monitor
  rclcpp sensor_msgs std_msgs rqt_gui rqt_gui_cpp qt_gui_cpp pluginlib
)

install(TARGETS imu_dds_monitor DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(rqt_gui plugin.xml)

ament_package()
```

- [ ] **Step 6: Verificar que el workspace compila (vacío)**

```bash
cd ~/ros2_ws
colcon build --packages-select imu_dds_adapter imu_dds_monitor 2>&1 | tail -20
```

Expected: errores de archivos fuente faltantes — normal en este punto.

- [ ] **Step 7: Commit**

```bash
cd ~/ros2_ws/src
git add imu_dds_adapter/CMakeLists.txt imu_dds_adapter/package.xml
git add imu_dds_monitor/CMakeLists.txt imu_dds_monitor/package.xml
git commit -m "feat: workspace scaffolding — CMakeLists y package.xml de ambos paquetes"
```

---

## Task 2: ImuFrame + NmeaParser (TDD)

**Files:**
- Create: `imu_dds_adapter/include/imu_dds_adapter/imu_frame.hpp`
- Create: `imu_dds_adapter/include/imu_dds_adapter/nmea_parser.hpp`
- Create: `imu_dds_adapter/src/nmea_parser.cpp`
- Create: `imu_dds_adapter/test/test_nmea_parser.cpp`

- [ ] **Step 1: Crear `imu_frame.hpp`**

```cpp
// include/imu_dds_adapter/imu_frame.hpp
#pragma once
#include <cstdint>

namespace imu_dds_adapter {

struct ImuFrame {
    // IMU Group — Registro 54 ($VNIMU)
    double mag_x{0.0}, mag_y{0.0}, mag_z{0.0};         // Gauss
    double accel_x{0.0}, accel_y{0.0}, accel_z{0.0};   // m/s²
    double gyro_x{0.0}, gyro_y{0.0}, gyro_z{0.0};      // rad/s
    double temperature{0.0};                             // °C
    double pressure{0.0};                                // kPa

    // Attitude — Registro 9 ($VNRRG,09), polling
    double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
    bool quat_valid{false};

    uint64_t timestamp_ns{0};
};

} // namespace imu_dds_adapter
```

- [ ] **Step 2: Escribir tests fallidos para NmeaParser**

```cpp
// test/test_nmea_parser.cpp
#include <gtest/gtest.h>
#include "imu_dds_adapter/nmea_parser.hpp"
#include "imu_dds_adapter/imu_frame.hpp"
#include <cstdint>
#include <cstdio>
#include <string>

using namespace imu_dds_adapter;

// Helper: computa checksum XOR para construir mensajes de test
static std::string makeMsg(const std::string& payload) {
    uint8_t cs = 0;
    for (char c : payload) cs ^= static_cast<uint8_t>(c);
    char buf[3];
    snprintf(buf, sizeof(buf), "%02X", cs);
    return "$" + payload + "*" + buf;
}

TEST(NmeaParser, ValidChecksumPasses) {
    // Mensaje real capturado del sensor
    EXPECT_TRUE(NmeaParser::validateChecksum("$VNYPR,-129.867,-000.355,-000.296*61"));
}

TEST(NmeaParser, InvalidChecksumFails) {
    EXPECT_FALSE(NmeaParser::validateChecksum("$VNYPR,-129.867,-000.355,-000.296*62"));
}

TEST(NmeaParser, MissingDollarFails) {
    EXPECT_FALSE(NmeaParser::validateChecksum("VNYPR,-129.867*61"));
}

TEST(NmeaParser, MissingStarFails) {
    EXPECT_FALSE(NmeaParser::validateChecksum("$VNYPR,-129.867"));
}

TEST(NmeaParser, ParsesVnimuFields) {
    std::string payload =
        "VNIMU,-00.1462,+00.1761,+00.2623,"
        "-00.0780,+00.0700,-09.8010,"
        "+00.007910,-00.003443,-00.009322,"
        "+36.3,+100.871";
    std::string line = makeMsg(payload);

    auto result = NmeaParser::parseVnimu(line);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->mag_x,    -0.1462,   1e-4);
    EXPECT_NEAR(result->mag_y,    +0.1761,   1e-4);
    EXPECT_NEAR(result->mag_z,    +0.2623,   1e-4);
    EXPECT_NEAR(result->accel_x,  -0.0780,   1e-4);
    EXPECT_NEAR(result->accel_y,  +0.0700,   1e-4);
    EXPECT_NEAR(result->accel_z,  -9.8010,   1e-3);
    EXPECT_NEAR(result->gyro_x,   +0.007910, 1e-6);
    EXPECT_NEAR(result->gyro_y,   -0.003443, 1e-6);
    EXPECT_NEAR(result->gyro_z,   -0.009322, 1e-6);
    EXPECT_NEAR(result->temperature, 36.3,   0.05);
    EXPECT_NEAR(result->pressure,  100.871,  1e-3);
}

TEST(NmeaParser, ParseVnimuRejectsWrongHeader) {
    std::string payload = "VNYPR,-129.867,-000.355,-000.296";
    std::string line = makeMsg(payload);
    EXPECT_FALSE(NmeaParser::parseVnimu(line).has_value());
}

TEST(NmeaParser, ParseVnimuRejectsInvalidChecksum) {
    std::string line = "$VNIMU,-00.1462,+00.1761,+00.2623,"
                       "-00.0780,+00.0700,-09.8010,"
                       "+00.007910,-00.003443,-00.009322,"
                       "+36.3,+100.871*FF";
    EXPECT_FALSE(NmeaParser::parseVnimu(line).has_value());
}

TEST(NmeaParser, ParseVnimuRejectsTooFewFields) {
    std::string payload = "VNIMU,-00.1462,+00.1761";
    std::string line = makeMsg(payload);
    EXPECT_FALSE(NmeaParser::parseVnimu(line).has_value());
}

TEST(NmeaParser, ParsesQuaternionFromVnrrg09) {
    std::string payload = "VNRRG,09,+0.004846,+0.000221,+0.905755,-0.423774";
    std::string line = makeMsg(payload);

    ImuFrame frame;
    EXPECT_TRUE(NmeaParser::parseQuaternion(line, frame));
    EXPECT_NEAR(frame.qx, +0.004846,  1e-6);
    EXPECT_NEAR(frame.qy, +0.000221,  1e-6);
    EXPECT_NEAR(frame.qz, +0.905755,  1e-6);
    EXPECT_NEAR(frame.qw, -0.423774,  1e-6);
    EXPECT_TRUE(frame.quat_valid);
}

TEST(NmeaParser, ParseQuaternionRejectsWrongHeader) {
    std::string payload = "VNRRG,08,-129.847,-000.510,-000.211";
    std::string line = makeMsg(payload);
    ImuFrame frame;
    EXPECT_FALSE(NmeaParser::parseQuaternion(line, frame));
    EXPECT_FALSE(frame.quat_valid);
}
```

- [ ] **Step 3: Ejecutar tests — verificar que fallan**

```bash
cd ~/ros2_ws
colcon build --packages-select imu_dds_adapter 2>&1 | grep -E "error|warning" | head -20
```

Expected: errores de compilación porque nmea_parser.hpp/cpp no existen.

- [ ] **Step 4: Crear `nmea_parser.hpp`**

```cpp
// include/imu_dds_adapter/nmea_parser.hpp
#pragma once
#include "imu_dds_adapter/imu_frame.hpp"
#include <optional>
#include <string>

namespace imu_dds_adapter {

class NmeaParser {
public:
    // Valida checksum XOR 8-bit. Formato: $PAYLOAD*HH
    static bool validateChecksum(const std::string& line);

    // Parsea mensaje $VNIMU (Registro 54 async output).
    // Retorna nullopt si checksum inválido, header incorrecto o campos insuficientes.
    static std::optional<ImuFrame> parseVnimu(const std::string& line);

    // Parsea respuesta $VNRRG,09 (quaternion polling).
    // Escribe qx,qy,qz,qw en frame y setea quat_valid=true.
    // Retorna false si el mensaje no es VNRRG,09 o tiene checksum inválido.
    static bool parseQuaternion(const std::string& line, ImuFrame& frame);

private:
    // Divide línea en tokens por delimitador. Excluye el prefijo $ y el *CS final.
    static std::vector<std::string> tokenize(const std::string& line);
};

} // namespace imu_dds_adapter
```

- [ ] **Step 5: Crear `nmea_parser.cpp`**

```cpp
// src/nmea_parser.cpp
#include "imu_dds_adapter/nmea_parser.hpp"
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <vector>

namespace imu_dds_adapter {

bool NmeaParser::validateChecksum(const std::string& line) {
    if (line.empty() || line.front() != '$') return false;
    auto star = line.rfind('*');
    if (star == std::string::npos || star + 3 > line.size()) return false;

    uint8_t computed = 0;
    for (size_t i = 1; i < star; ++i) {
        computed ^= static_cast<uint8_t>(line[i]);
    }

    try {
        uint8_t expected = static_cast<uint8_t>(
            std::stoul(line.substr(star + 1, 2), nullptr, 16));
        return computed == expected;
    } catch (...) {
        return false;
    }
}

std::vector<std::string> NmeaParser::tokenize(const std::string& line) {
    // Extrae el payload: entre $ y *
    auto start = line.find('$');
    auto end   = line.rfind('*');
    if (start == std::string::npos || end == std::string::npos) return {};

    std::string payload = line.substr(start + 1, end - start - 1);
    std::vector<std::string> tokens;
    std::stringstream ss(payload);
    std::string token;
    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }
    return tokens;
}

std::optional<ImuFrame> NmeaParser::parseVnimu(const std::string& line) {
    if (!validateChecksum(line)) return std::nullopt;

    auto tokens = tokenize(line);
    // tokens[0] = "VNIMU", tokens[1..11] = 11 campos de datos
    if (tokens.size() < 12 || tokens[0] != "VNIMU") return std::nullopt;

    try {
        ImuFrame f;
        f.mag_x       = std::stod(tokens[1]);
        f.mag_y       = std::stod(tokens[2]);
        f.mag_z       = std::stod(tokens[3]);
        f.accel_x     = std::stod(tokens[4]);
        f.accel_y     = std::stod(tokens[5]);
        f.accel_z     = std::stod(tokens[6]);
        f.gyro_x      = std::stod(tokens[7]);
        f.gyro_y      = std::stod(tokens[8]);
        f.gyro_z      = std::stod(tokens[9]);
        f.temperature = std::stod(tokens[10]);
        f.pressure    = std::stod(tokens[11]);
        return f;
    } catch (...) {
        return std::nullopt;
    }
}

bool NmeaParser::parseQuaternion(const std::string& line, ImuFrame& frame) {
    if (!validateChecksum(line)) return false;

    auto tokens = tokenize(line);
    // tokens[0]="VNRRG", tokens[1]="09", tokens[2..5]=qx,qy,qz,qw
    if (tokens.size() < 6 || tokens[0] != "VNRRG" || tokens[1] != "09") return false;

    try {
        frame.qx         = std::stod(tokens[2]);
        frame.qy         = std::stod(tokens[3]);
        frame.qz         = std::stod(tokens[4]);
        frame.qw         = std::stod(tokens[5]);
        frame.quat_valid = true;
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace imu_dds_adapter
```

- [ ] **Step 6: Ejecutar tests — verificar que pasan**

```bash
cd ~/ros2_ws
colcon build --packages-select imu_dds_adapter && \
colcon test --packages-select imu_dds_adapter --event-handlers console_direct+ 2>&1 | grep -E "PASS|FAIL|OK|error"
```

Expected: todos los tests de `test_nmea_parser` en PASS.

- [ ] **Step 7: Commit**

```bash
cd ~/ros2_ws/src
git add imu_dds_adapter/include/imu_dds_adapter/imu_frame.hpp
git add imu_dds_adapter/include/imu_dds_adapter/nmea_parser.hpp
git add imu_dds_adapter/src/nmea_parser.cpp
git add imu_dds_adapter/test/test_nmea_parser.cpp
git commit -m "feat: ImuFrame DTO y NmeaParser con tests — parseVnimu y parseQuaternion"
```

---

## Task 3: ImuConverter (TDD)

**Files:**
- Create: `imu_dds_adapter/include/imu_dds_adapter/imu_converter.hpp`
- Create: `imu_dds_adapter/src/imu_converter.cpp`
- Create: `imu_dds_adapter/test/test_imu_converter.cpp`

- [ ] **Step 1: Escribir tests fallidos para ImuConverter**

```cpp
// test/test_imu_converter.cpp
#include <gtest/gtest.h>
#include "imu_dds_adapter/imu_converter.hpp"
#include "imu_dds_adapter/imu_frame.hpp"
#include <cmath>

using namespace imu_dds_adapter;

static ImuFrame makeFrame() {
    ImuFrame f;
    f.accel_x = -0.078; f.accel_y = 0.070; f.accel_z = -9.801;
    f.gyro_x  =  0.0079; f.gyro_y = -0.0034; f.gyro_z = -0.0093;
    f.mag_x   = -0.1462; f.mag_y  =  0.1761; f.mag_z  =  0.2623;
    f.temperature = 36.3; f.pressure = 100.871;
    f.qx = 0.004846; f.qy = 0.000221; f.qz = 0.905755; f.qw = -0.423774;
    f.quat_valid = true;
    return f;
}

TEST(ImuConverter, LinearAccelerationMapsDirectly) {
    auto msg = ImuConverter::convert(makeFrame(), "imu_link");
    EXPECT_NEAR(msg.linear_acceleration.x, -0.078,  1e-4);
    EXPECT_NEAR(msg.linear_acceleration.y,  0.070,  1e-4);
    EXPECT_NEAR(msg.linear_acceleration.z, -9.801,  1e-3);
}

TEST(ImuConverter, AngularVelocityMapsDirectly) {
    auto msg = ImuConverter::convert(makeFrame(), "imu_link");
    EXPECT_NEAR(msg.angular_velocity.x,  0.0079, 1e-5);
    EXPECT_NEAR(msg.angular_velocity.y, -0.0034, 1e-5);
    EXPECT_NEAR(msg.angular_velocity.z, -0.0093, 1e-5);
}

TEST(ImuConverter, OrientationQuaternionIsUnitVector) {
    auto msg = ImuConverter::convert(makeFrame(), "imu_link");
    double norm = std::sqrt(
        msg.orientation.x * msg.orientation.x +
        msg.orientation.y * msg.orientation.y +
        msg.orientation.z * msg.orientation.z +
        msg.orientation.w * msg.orientation.w
    );
    EXPECT_NEAR(norm, 1.0, 1e-6);
}

TEST(ImuConverter, FrameIdIsSet) {
    auto msg = ImuConverter::convert(makeFrame(), "imu_link");
    EXPECT_EQ(msg.header.frame_id, "imu_link");
}

TEST(ImuConverter, CovarianceUnknownWhenQuatInvalid) {
    ImuFrame f = makeFrame();
    f.quat_valid = false;
    auto msg = ImuConverter::convert(f, "imu_link");
    EXPECT_DOUBLE_EQ(msg.orientation_covariance[0], -1.0);
}

TEST(ImuConverter, CovarianceDiagonalWhenQuatValid) {
    auto msg = ImuConverter::convert(makeFrame(), "imu_link");
    // Diagonal no cero, off-diagonal cero
    EXPECT_GT(msg.orientation_covariance[0], 0.0);
    EXPECT_DOUBLE_EQ(msg.orientation_covariance[1], 0.0);
    EXPECT_GT(msg.angular_velocity_covariance[0], 0.0);
    EXPECT_DOUBLE_EQ(msg.angular_velocity_covariance[1], 0.0);
    EXPECT_GT(msg.linear_acceleration_covariance[0], 0.0);
    EXPECT_DOUBLE_EQ(msg.linear_acceleration_covariance[1], 0.0);
}
```

- [ ] **Step 2: Ejecutar tests — verificar que fallan**

```bash
cd ~/ros2_ws
colcon build --packages-select imu_dds_adapter 2>&1 | grep "error:" | head -5
```

Expected: error de compilación — imu_converter.hpp no existe.

- [ ] **Step 3: Crear `imu_converter.hpp`**

```cpp
// include/imu_dds_adapter/imu_converter.hpp
#pragma once
#include "imu_dds_adapter/imu_frame.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <string>

namespace imu_dds_adapter {

class ImuConverter {
public:
    // Convierte ImuFrame a sensor_msgs/Imu.
    // Aplica rotación NED→ENU al cuaternión de orientación.
    // Covarianzas de orientación son -1 si frame.quat_valid == false.
    static sensor_msgs::msg::Imu convert(
        const ImuFrame& frame,
        const std::string& frame_id);

private:
    // Varianzas del VN-100S-CR (de datasheet)
    static constexpr double kGyroVar  = 1.2e-7;  // rad²/s²
    static constexpr double kAccelVar = 4.0e-5;  // m²/s⁴
    static constexpr double kOrientVar= 1.0e-4;  // rad²
};

} // namespace imu_dds_adapter
```

- [ ] **Step 4: Crear `imu_converter.cpp`**

```cpp
// src/imu_converter.cpp
#include "imu_dds_adapter/imu_converter.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace imu_dds_adapter {

sensor_msgs::msg::Imu ImuConverter::convert(
    const ImuFrame& frame,
    const std::string& frame_id)
{
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = frame_id;
    // stamp se asigna en ImuPublisherNode con rclcpp::Clock

    // Linear acceleration — body frame, sin conversión de unidades
    msg.linear_acceleration.x = frame.accel_x;
    msg.linear_acceleration.y = frame.accel_y;
    msg.linear_acceleration.z = frame.accel_z;
    msg.linear_acceleration_covariance = {
        kAccelVar, 0, 0,
        0, kAccelVar, 0,
        0, 0, kAccelVar
    };

    // Angular velocity — body frame, sin conversión de unidades
    msg.angular_velocity.x = frame.gyro_x;
    msg.angular_velocity.y = frame.gyro_y;
    msg.angular_velocity.z = frame.gyro_z;
    msg.angular_velocity_covariance = {
        kGyroVar, 0, 0,
        0, kGyroVar, 0,
        0, 0, kGyroVar
    };

    // Orientation — cuaternión con rotación NED→ENU
    if (!frame.quat_valid) {
        // Covarianza -1 indica orientación desconocida (REP-145)
        msg.orientation_covariance[0] = -1.0;
    } else {
        // NED→ENU: rotación 180° alrededor del eje (1/√2, 1/√2, 0)
        // Quaternion: (w=0, x=1/√2, y=1/√2, z=0)
        static const tf2::Quaternion kNedToEnu(
            1.0 / std::sqrt(2.0),  // x
            1.0 / std::sqrt(2.0),  // y
            0.0,                    // z
            0.0                     // w
        );
        tf2::Quaternion q_ned(frame.qx, frame.qy, frame.qz, frame.qw);
        tf2::Quaternion q_enu = kNedToEnu * q_ned;
        q_enu.normalize();

        msg.orientation.x = q_enu.x();
        msg.orientation.y = q_enu.y();
        msg.orientation.z = q_enu.z();
        msg.orientation.w = q_enu.w();
        msg.orientation_covariance = {
            kOrientVar, 0, 0,
            0, kOrientVar, 0,
            0, 0, kOrientVar
        };
    }

    return msg;
}

} // namespace imu_dds_adapter
```

- [ ] **Step 5: Ejecutar tests — verificar que pasan**

```bash
cd ~/ros2_ws
colcon build --packages-select imu_dds_adapter && \
colcon test --packages-select imu_dds_adapter --event-handlers console_direct+ 2>&1 | grep -E "PASS|FAIL|OK"
```

Expected: `test_nmea_parser` y `test_imu_converter` — todos en PASS.

- [ ] **Step 6: Commit**

```bash
cd ~/ros2_ws/src
git add imu_dds_adapter/include/imu_dds_adapter/imu_converter.hpp
git add imu_dds_adapter/src/imu_converter.cpp
git add imu_dds_adapter/test/test_imu_converter.cpp
git commit -m "feat: ImuConverter con rotacion NED→ENU y matrices de covarianza — tests OK"
```

---

## Task 4: SerialPort

**Files:**
- Create: `imu_dds_adapter/include/imu_dds_adapter/serial_port.hpp`
- Create: `imu_dds_adapter/src/serial_port.cpp`

No hay tests unitarios automáticos para SerialPort (requiere hardware). La verificación es manual en Task 7.

- [ ] **Step 1: Crear `serial_port.hpp`**

```cpp
// include/imu_dds_adapter/serial_port.hpp
#pragma once
#include <string>
#include <stdexcept>

namespace imu_dds_adapter {

class SerialPortException : public std::runtime_error {
public:
    explicit SerialPortException(const std::string& msg) : std::runtime_error(msg) {}
};

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    // Abre el puerto y configura baud rate.
    // Lanza SerialPortException si falla.
    void open(const std::string& device, int baud_rate);

    // Envía la secuencia de configuración al VN-100:
    //   $VNASY,0    — pausa stream
    //   $VNWRG,6,19 — output type = VNIMU (Registro 54)
    //   $VNWRG,7,<hz> — frecuencia
    //   $VNASY,1    — reanuda stream
    void configureSensor(int output_hz);

    // Lee una línea completa terminada en \n. Bloqueante.
    // Lanza SerialPortException si timeout (500ms sin datos).
    std::string readline();

    // Escribe un comando ASCII al sensor. Agrega \r\n automáticamente.
    void write(const std::string& cmd);

    void close();
    bool isOpen() const { return fd_ >= 0; }

private:
    int fd_{-1};

    // Convierte baud rate numérico a constante termios (B115200, etc.)
    static speed_t toBaudRate(int baud);
};

} // namespace imu_dds_adapter
```

- [ ] **Step 2: Crear `serial_port.cpp`**

```cpp
// src/serial_port.cpp
#include "imu_dds_adapter/serial_port.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <sstream>
#include <thread>
#include <chrono>

namespace imu_dds_adapter {

SerialPort::~SerialPort() { close(); }

speed_t SerialPort::toBaudRate(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 921600: return B921600;
        default:
            throw SerialPortException("Baud rate no soportado: " + std::to_string(baud));
    }
}

void SerialPort::open(const std::string& device, int baud_rate) {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        throw SerialPortException("No se pudo abrir " + device + ": " + strerror(errno));
    }

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_); fd_ = -1;
        throw SerialPortException("tcgetattr falló: " + std::string(strerror(errno)));
    }

    speed_t speed = toBaudRate(baud_rate);
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8 bits
    tty.c_cflag &= ~(PARENB | CSTOPB);             // sin paridad, 1 stop bit
    tty.c_cflag |= (CLOCAL | CREAD);               // ignorar modem, habilitar lectura
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);        // sin control de flujo SW
    tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP);
    tty.c_oflag  = 0;                               // raw output
    tty.c_lflag  = 0;                               // raw input (sin canonical, sin echo)
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;  // timeout 0.5s por carácter

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_); fd_ = -1;
        throw SerialPortException("tcsetattr falló: " + std::string(strerror(errno)));
    }
    tcflush(fd_, TCIFLUSH);
}

void SerialPort::write(const std::string& cmd) {
    std::string full = cmd + "\r\n";
    ssize_t written = ::write(fd_, full.c_str(), full.size());
    if (written < 0) {
        throw SerialPortException("Error escribiendo al puerto: " + std::string(strerror(errno)));
    }
}

void SerialPort::configureSensor(int output_hz) {
    using namespace std::chrono_literals;

    write("$VNASY,0*XX");                                     // pausa stream
    std::this_thread::sleep_for(200ms);

    write("$VNWRG,6,19*XX");                                  // Registro 54 = VNIMU
    std::this_thread::sleep_for(100ms);

    write("$VNWRG,7," + std::to_string(output_hz) + "*XX");   // frecuencia
    std::this_thread::sleep_for(100ms);

    write("$VNASY,1*XX");                                     // reanuda stream
    std::this_thread::sleep_for(200ms);

    tcflush(fd_, TCIFLUSH);  // descarta buffer acumulado durante la configuración
}

std::string SerialPort::readline() {
    std::string line;
    char ch;

    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;

    while (true) {
        int ret = poll(&pfd, 1, 500);  // timeout 500ms
        if (ret < 0) {
            throw SerialPortException("poll falló: " + std::string(strerror(errno)));
        }
        if (ret == 0) {
            throw SerialPortException("Timeout leyendo del puerto serial");
        }

        ssize_t n = ::read(fd_, &ch, 1);
        if (n < 0) {
            throw SerialPortException("Error leyendo del puerto: " + std::string(strerror(errno)));
        }
        if (n == 0) continue;

        if (ch == '\n') return line;
        if (ch != '\r') line += ch;
    }
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

} // namespace imu_dds_adapter
```

- [ ] **Step 3: Verificar compilación**

```bash
cd ~/ros2_ws
colcon build --packages-select imu_dds_adapter 2>&1 | grep -E "error:|warning:" | head -10
```

Expected: sin errores de compilación en los archivos nuevos.

- [ ] **Step 4: Commit**

```bash
cd ~/ros2_ws/src
git add imu_dds_adapter/include/imu_dds_adapter/serial_port.hpp
git add imu_dds_adapter/src/serial_port.cpp
git commit -m "feat: SerialPort — UART driver con termios, configureSensor y readline"
```

---

## Task 5: ImuPublisherNode

**Files:**
- Create: `imu_dds_adapter/include/imu_dds_adapter/imu_publisher_node.hpp`
- Create: `imu_dds_adapter/src/imu_publisher_node.cpp`

- [ ] **Step 1: Crear `imu_publisher_node.hpp`**

```cpp
// include/imu_dds_adapter/imu_publisher_node.hpp
#pragma once
#include "imu_dds_adapter/serial_port.hpp"
#include "imu_dds_adapter/nmea_parser.hpp"
#include "imu_dds_adapter/imu_converter.hpp"
#include "imu_dds_adapter/imu_frame.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <string>

namespace imu_dds_adapter {

class ImuPublisherNode : public rclcpp::Node {
public:
    explicit ImuPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ImuPublisherNode() override;

private:
    // Thread A: lee líneas del serial y las encola
    void serialReaderThread();

    // Thread B (Timer callback): desencola, parsea, publica
    void timerCallback();

    // Polling del cuaternión cada quat_poll_every_n_ ciclos
    void pollQuaternion();

    // Publica el transform estático map → imu_link
    void publishStaticTransform();

    // Publica diagnóstico en /imu/status
    void publishDiagnostics();

    // Intenta reconfigurar el sensor ante timeout (máx. 3 intentos)
    bool tryReconfigure();

    // ── Parámetros ──────────────────────────────────────────────
    std::string serial_port_;
    int         baud_rate_;
    int         output_hz_;
    int         quat_poll_every_n_;
    std::string frame_id_;
    bool        reconfigure_sensor_;

    // ── Objetos de comunicación ──────────────────────────────────
    SerialPort  serial_;

    // ── Threading ────────────────────────────────────────────────
    std::thread reader_thread_;
    std::atomic<bool> running_{false};

    std::queue<std::string> frame_queue_;
    std::mutex queue_mutex_;

    ImuFrame quat_cache_;
    std::mutex quat_mutex_;
    int quat_poll_counter_{0};

    // ── Publishers ───────────────────────────────────────────────
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr           imu_raw_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           raw_ascii_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    // ── Métricas de diagnóstico ──────────────────────────────────
    std::atomic<uint64_t> frames_received_{0};
    std::atomic<uint64_t> frames_dropped_{0};
    std::atomic<uint64_t> checksum_errors_{0};
};

} // namespace imu_dds_adapter
```

- [ ] **Step 2: Crear `imu_publisher_node.cpp`**

```cpp
// src/imu_publisher_node.cpp
#include "imu_dds_adapter/imu_publisher_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

using namespace std::chrono_literals;

namespace imu_dds_adapter {

ImuPublisherNode::ImuPublisherNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("imu_dds_adapter", options)
{
    // ── Declarar y leer parámetros ───────────────────────────────
    serial_port_         = declare_parameter<std::string>("serial_port",    "/dev/ttyUSB0");
    baud_rate_           = declare_parameter<int>        ("baud_rate",      115200);
    output_hz_           = declare_parameter<int>        ("output_hz",      50);
    quat_poll_every_n_   = declare_parameter<int>        ("quat_poll_every_n", 5);
    frame_id_            = declare_parameter<std::string>("frame_id",       "imu_link");
    reconfigure_sensor_  = declare_parameter<bool>       ("reconfigure_sensor", true);

    // ── Publishers ───────────────────────────────────────────────
    imu_pub_      = create_publisher<sensor_msgs::msg::Imu>("/imu/data",     10);
    imu_raw_pub_  = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
    raw_ascii_pub_= create_publisher<std_msgs::msg::String>("/imu/raw_ascii",10);
    status_pub_   = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/imu/status", 1);

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // ── Abrir puerto y configurar sensor ────────────────────────
    RCLCPP_INFO(get_logger(), "Abriendo %s a %d baud", serial_port_.c_str(), baud_rate_);
    try {
        serial_.open(serial_port_, baud_rate_);
        if (reconfigure_sensor_) {
            RCLCPP_INFO(get_logger(), "Configurando sensor: VNIMU a %d Hz", output_hz_);
            serial_.configureSensor(output_hz_);
        }
    } catch (const SerialPortException& e) {
        RCLCPP_FATAL(get_logger(), "Error de puerto serial: %s", e.what());
        throw;
    }

    publishStaticTransform();

    // ── Iniciar thread lector ────────────────────────────────────
    running_ = true;
    reader_thread_ = std::thread(&ImuPublisherNode::serialReaderThread, this);

    // ── Timer principal @ output_hz_ ────────────────────────────
    auto period = std::chrono::duration<double>(1.0 / output_hz_);
    timer_ = create_wall_timer(period, [this]() { timerCallback(); });

    // ── Timer de diagnóstico @ 1 Hz ─────────────────────────────
    diag_timer_ = create_wall_timer(1s, [this]() { publishDiagnostics(); });

    RCLCPP_INFO(get_logger(), "IMU-DDS Adapter iniciado en %s", serial_port_.c_str());
}

ImuPublisherNode::~ImuPublisherNode() {
    running_ = false;
    serial_.close();
    if (reader_thread_.joinable()) reader_thread_.join();
}

void ImuPublisherNode::serialReaderThread() {
    while (running_) {
        try {
            std::string line = serial_.readline();

            // Publicar raw ASCII
            auto raw_msg = std_msgs::msg::String();
            raw_msg.data = line;
            raw_ascii_pub_->publish(raw_msg);

            // Intentar parsear como cuaternión polling
            {
                std::lock_guard<std::mutex> lock(quat_mutex_);
                if (NmeaParser::parseQuaternion(line, quat_cache_)) continue;
            }

            // Validar checksum y encolar
            if (!NmeaParser::validateChecksum(line)) {
                checksum_errors_++;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Checksum inválido: %s", line.c_str());
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (frame_queue_.size() > 10) {
                    frame_queue_.pop();  // descarta el más antiguo
                    frames_dropped_++;
                }
                frame_queue_.push(line);
            }
            frames_received_++;

        } catch (const SerialPortException& e) {
            if (!running_) break;
            RCLCPP_ERROR(get_logger(), "Timeout serial: %s. Intentando reconfigurar...", e.what());
            if (!tryReconfigure()) {
                RCLCPP_FATAL(get_logger(), "No se pudo recuperar la conexión. Cerrando.");
                rclcpp::shutdown();
                break;
            }
        }
    }
}

void ImuPublisherNode::timerCallback() {
    std::string line;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (frame_queue_.empty()) return;
        line = frame_queue_.front();
        frame_queue_.pop();
    }

    auto frame_opt = NmeaParser::parseVnimu(line);
    if (!frame_opt) return;

    ImuFrame frame = *frame_opt;
    frame.timestamp_ns = now().nanoseconds();

    // Copiar cuaternión cacheado
    {
        std::lock_guard<std::mutex> lock(quat_mutex_);
        frame.qx         = quat_cache_.qx;
        frame.qy         = quat_cache_.qy;
        frame.qz         = quat_cache_.qz;
        frame.qw         = quat_cache_.qw;
        frame.quat_valid = quat_cache_.quat_valid;
    }

    // Publicar datos compensados
    auto imu_msg = ImuConverter::convert(frame, frame_id_);
    imu_msg.header.stamp = now();
    imu_pub_->publish(imu_msg);

    // Publicar datos sin compensar en /imu/data_raw
    // (en este nivel ASCII, accel/gyro ya son compensados por el filtro del sensor;
    //  data_raw lleva los mismos datos para demostración del topic secundario)
    imu_raw_pub_->publish(imu_msg);

    // Polling del cuaternión
    quat_poll_counter_++;
    if (quat_poll_counter_ >= quat_poll_every_n_) {
        quat_poll_counter_ = 0;
        pollQuaternion();
    }
}

void ImuPublisherNode::pollQuaternion() {
    try {
        serial_.write("$VNRRG,09*XX");
        // La respuesta llega en serialReaderThread → parseQuaternion → quat_cache_
    } catch (const SerialPortException& e) {
        RCLCPP_WARN(get_logger(), "Error en polling de cuaternión: %s", e.what());
    }
}

void ImuPublisherNode::publishStaticTransform() {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = now();
    tf.header.frame_id = "map";
    tf.child_frame_id  = frame_id_;
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf);
}

void ImuPublisherNode::publishDiagnostics() {
    diagnostic_msgs::msg::DiagnosticStatus msg;
    msg.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
    msg.name    = "VN-100 IMU Adapter";
    msg.message = "Streaming " + std::to_string(output_hz_) + " Hz";

    auto kv = [](const std::string& k, auto v) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key   = k;
        kv.value = std::to_string(v);
        return kv;
    };

    msg.values.push_back(kv("frames_received",  frames_received_.load()));
    msg.values.push_back(kv("frames_dropped",   frames_dropped_.load()));
    msg.values.push_back(kv("checksum_errors",  checksum_errors_.load()));

    status_pub_->publish(msg);
}

bool ImuPublisherNode::tryReconfigure() {
    for (int i = 0; i < 3; ++i) {
        RCLCPP_WARN(get_logger(), "Intento de reconfiguración %d/3", i + 1);
        try {
            serial_.configureSensor(output_hz_);
            return true;
        } catch (...) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    return false;
}

} // namespace imu_dds_adapter
```

- [ ] **Step 3: Verificar compilación**

```bash
cd ~/ros2_ws
colcon build --packages-select imu_dds_adapter 2>&1 | grep -E "^.*(error|Finished)" | head -10
```

Expected: `Finished <<< imu_dds_adapter`

- [ ] **Step 4: Commit**

```bash
cd ~/ros2_ws/src
git add imu_dds_adapter/include/imu_dds_adapter/imu_publisher_node.hpp
git add imu_dds_adapter/src/imu_publisher_node.cpp
git commit -m "feat: ImuPublisherNode — threading, QoS, polling cuaternion, diagnosticos"
```

---

## Task 6: main.cpp, Parámetros y Launch File

**Files:**
- Create: `imu_dds_adapter/src/main.cpp`
- Create: `imu_dds_adapter/config/adapter_params.yaml`
- Create: `imu_dds_adapter/launch/imu_dds.launch.py`

- [ ] **Step 1: Crear `main.cpp`**

```cpp
// src/main.cpp
#include "imu_dds_adapter/imu_publisher_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<imu_dds_adapter::ImuPublisherNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Excepción fatal: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
```

- [ ] **Step 2: Crear `config/adapter_params.yaml`**

```yaml
imu_dds_adapter:
  ros__parameters:
    serial_port: /dev/ttyUSB0
    baud_rate: 115200
    output_hz: 50
    quat_poll_every_n: 5
    frame_id: imu_link
    reconfigure_sensor: true
```

- [ ] **Step 3: Crear `launch/imu_dds.launch.py`**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('imu_dds_adapter'),
        'config', 'adapter_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='imu_dds_adapter',
            executable='imu_dds_adapter_node',
            name='imu_dds_adapter',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        )
    ])
```

- [ ] **Step 4: Build y verificar ejecutable**

```bash
cd ~/ros2_ws
colcon build --packages-select imu_dds_adapter
source install/setup.bash
ls install/imu_dds_adapter/lib/imu_dds_adapter/
```

Expected: `imu_dds_adapter_node` presente.

- [ ] **Step 5: Commit**

```bash
cd ~/ros2_ws/src
git add imu_dds_adapter/src/main.cpp
git add imu_dds_adapter/config/adapter_params.yaml
git add imu_dds_adapter/launch/imu_dds.launch.py
git commit -m "feat: main.cpp, adapter_params.yaml y launch file"
```

---

## Task 7: Test de Integración con Sensor Real

Verificación end-to-end: sensor → serial → DDS → topics ROS 2.

- [ ] **Step 1: Verificar que el sensor está conectado**

```bash
ls -la /dev/ttyUSB0
# Expected: crw-rw---- 1 root dialout 188, 0 ...
```

- [ ] **Step 2: Lanzar el adaptador**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch imu_dds_adapter imu_dds.launch.py
```

Expected en stdout:
```
[imu_dds_adapter]: Abriendo /dev/ttyUSB0 a 115200 baud
[imu_dds_adapter]: Configurando sensor: VNIMU a 50 Hz
[imu_dds_adapter]: IMU-DDS Adapter iniciado en /dev/ttyUSB0
```

- [ ] **Step 3: En otra terminal — verificar topics activos**

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep imu
```

Expected:
```
/imu/data
/imu/data_raw
/imu/raw_ascii
/imu/status
```

- [ ] **Step 4: Verificar frecuencia de publicación**

```bash
ros2 topic hz /imu/data
```

Expected: `average rate: 50.xxx`

- [ ] **Step 5: Inspeccionar un mensaje /imu/data**

```bash
ros2 topic echo /imu/data --once
```

Expected: mensaje con `linear_acceleration.z ≈ -9.8`, `header.frame_id: imu_link`, quaternion con `‖q‖ ≈ 1.0`.

- [ ] **Step 6: Verificar raw ASCII**

```bash
ros2 topic echo /imu/raw_ascii --once
```

Expected: `data: '$VNIMU,-00.xxxx,...'`

- [ ] **Step 7: Verificar diagnóstico**

```bash
ros2 topic echo /imu/status --once
```

Expected: `level: 0` (OK), `name: VN-100 IMU Adapter`, `frames_received > 0`.

- [ ] **Step 8: Commit**

```bash
cd ~/ros2_ws/src
git commit --allow-empty -m "test: integracion end-to-end verificada — 50Hz, topics OK, diagnosticos OK"
```

---

## Task 8: Plugin rqt — imu_dds_monitor

**Files:**
- Create: `imu_dds_monitor/plugin.xml`
- Create: `imu_dds_monitor/resource/imu_monitor.ui`
- Create: `imu_dds_monitor/include/imu_dds_monitor/imu_monitor_widget.hpp`
- Create: `imu_dds_monitor/src/imu_monitor_plugin.cpp`
- Create: `imu_dds_monitor/src/imu_monitor_widget.cpp`

- [ ] **Step 1: Crear `plugin.xml`**

```xml
<library path="imu_dds_monitor">
  <class name="imu_dds_monitor/ImuMonitorPlugin"
         type="imu_dds_monitor::ImuMonitorPlugin"
         base_class_type="rqt_gui_cpp::Plugin">
    <description>Monitor para el adaptador IMU-DDS — Raw ASCII, Parsed Fields, DDS Output</description>
    <qtgui>
      <group>
        <label>IMU</label>
      </group>
      <label>IMU DDS Monitor</label>
      <icon type="theme">utilities-system-monitor</icon>
      <statustip>Visualiza datos raw, parseados y DDS del sensor VN-100</statustip>
    </qtgui>
  </class>
</library>
```

- [ ] **Step 2: Crear `resource/imu_monitor.ui`**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<ui version="4.0">
 <class>ImuMonitorWidget</class>
 <widget class="QWidget" name="ImuMonitorWidget">
  <property name="windowTitle"><string>IMU DDS Monitor</string></property>
  <layout class="QVBoxLayout" name="main_layout">
   <item>
    <widget class="QTabWidget" name="tab_widget">
     <property name="currentIndex"><number>0</number></property>
     <widget class="QWidget" name="raw_tab">
      <attribute name="title"><string>Raw ASCII</string></attribute>
      <layout class="QVBoxLayout">
       <item>
        <widget class="QTextEdit" name="raw_text">
         <property name="readOnly"><bool>true</bool></property>
         <property name="fontFamily"><string>Monospace</string></property>
        </widget>
       </item>
      </layout>
     </widget>
     <widget class="QWidget" name="parsed_tab">
      <attribute name="title"><string>Parsed Fields</string></attribute>
      <layout class="QVBoxLayout">
       <item>
        <widget class="QTableWidget" name="parsed_table">
         <property name="columnCount"><number>2</number></property>
         <property name="rowCount"><number>11</number></property>
         <attribute name="horizontalHeaderItem" column="0">
          <item><property name="text"><string>Campo</string></property></item>
         </attribute>
         <attribute name="horizontalHeaderItem" column="1">
          <item><property name="text"><string>Valor</string></property></item>
         </attribute>
        </widget>
       </item>
      </layout>
     </widget>
     <widget class="QWidget" name="dds_tab">
      <attribute name="title"><string>DDS Output</string></attribute>
      <layout class="QVBoxLayout">
       <item>
        <widget class="QTableWidget" name="dds_table">
         <property name="columnCount"><number>2</number></property>
         <property name="rowCount"><number>12</number></property>
         <attribute name="horizontalHeaderItem" column="0">
          <item><property name="text"><string>Campo DDS</string></property></item>
         </attribute>
         <attribute name="horizontalHeaderItem" column="1">
          <item><property name="text"><string>Valor</string></property></item>
         </attribute>
        </widget>
       </item>
      </layout>
     </widget>
    </widget>
   </item>
   <item>
    <widget class="QLabel" name="status_label">
     <property name="text"><string>● Desconectado</string></property>
    </widget>
   </item>
  </layout>
 </widget>
</ui>
```

- [ ] **Step 3: Crear `include/imu_dds_monitor/imu_monitor_widget.hpp`**

```cpp
// include/imu_dds_monitor/imu_monitor_widget.hpp
#pragma once
#include <QWidget>
#include <QTimer>
#include <QTableWidget>
#include <QTextEdit>
#include <QLabel>
#include <QTabWidget>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>
#include <optional>

namespace Ui { class ImuMonitorWidget; }

namespace imu_dds_monitor {

class ImuMonitorWidget : public QWidget {
    Q_OBJECT
public:
    explicit ImuMonitorWidget(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
    ~ImuMonitorWidget() override;
    void shutdown();

private slots:
    void refreshUi();

private:
    void setupParsedTable();
    void setupDdsTable();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr    imu_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr    raw_sub_;

    std::mutex data_mutex_;
    std::optional<sensor_msgs::msg::Imu> last_imu_;
    std::string last_raw_;
    uint64_t msg_count_{0};

    QTimer* refresh_timer_;

    // UI elements (cargados desde .ui)
    QTabWidget*   tab_widget_;
    QTextEdit*    raw_text_;
    QTableWidget* parsed_table_;
    QTableWidget* dds_table_;
    QLabel*       status_label_;

    Ui::ImuMonitorWidget* ui_;
};

} // namespace imu_dds_monitor
```

- [ ] **Step 4: Crear `src/imu_monitor_widget.cpp`**

```cpp
// src/imu_monitor_widget.cpp
#include "imu_dds_monitor/imu_monitor_widget.hpp"
#include "ui_imu_monitor.h"  // generado por CMAKE_AUTOUIC desde resource/imu_monitor.ui

#include <QHeaderView>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace imu_dds_monitor {

ImuMonitorWidget::ImuMonitorWidget(rclcpp::Node::SharedPtr node, QWidget* parent)
: QWidget(parent), node_(node), ui_(new Ui::ImuMonitorWidget)
{
    ui_->setupUi(this);

    tab_widget_   = ui_->tab_widget;
    raw_text_     = ui_->raw_text;
    parsed_table_ = ui_->parsed_table;
    dds_table_    = ui_->dds_table;
    status_label_ = ui_->status_label;

    setupParsedTable();
    setupDdsTable();

    // Suscribir a topics DDS
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            last_imu_ = *msg;
            msg_count_++;
        });

    raw_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/imu/raw_ascii", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            last_raw_ = msg->data;
        });

    // Refrescar UI a 10 Hz desde hilo Qt
    refresh_timer_ = new QTimer(this);
    connect(refresh_timer_, &QTimer::timeout, this, &ImuMonitorWidget::refreshUi);
    refresh_timer_->start(100);  // 100ms = 10 Hz
}

ImuMonitorWidget::~ImuMonitorWidget() { delete ui_; }

void ImuMonitorWidget::shutdown() {
    refresh_timer_->stop();
    imu_sub_.reset();
    raw_sub_.reset();
}

void ImuMonitorWidget::setupParsedTable() {
    QStringList fields = {
        "accel_x [m/s²]", "accel_y [m/s²]", "accel_z [m/s²]",
        "gyro_x [rad/s]",  "gyro_y [rad/s]",  "gyro_z [rad/s]",
        "quat_x",          "quat_y",           "quat_z",          "quat_w",
        "frame_id"
    };
    parsed_table_->setRowCount(fields.size());
    for (int i = 0; i < fields.size(); ++i) {
        parsed_table_->setItem(i, 0, new QTableWidgetItem(fields[i]));
        parsed_table_->setItem(i, 1, new QTableWidgetItem("—"));
    }
    parsed_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

void ImuMonitorWidget::setupDdsTable() {
    QStringList fields = {
        "topic", "seq",
        "stamp.sec", "stamp.nanosec",
        "frame_id",
        "orient.x", "orient.y", "orient.z", "orient.w",
        "lin_acc.z [m/s²]",
        "ang_vel.x [rad/s]",
        "hz (aprox)"
    };
    dds_table_->setRowCount(fields.size());
    for (int i = 0; i < fields.size(); ++i) {
        dds_table_->setItem(i, 0, new QTableWidgetItem(fields[i]));
        dds_table_->setItem(i, 1, new QTableWidgetItem("—"));
    }
    dds_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

void ImuMonitorWidget::refreshUi() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Panel Raw ASCII
    if (!last_raw_.empty()) {
        raw_text_->append(QString::fromStdString(last_raw_));
        // Mantener solo las últimas 50 líneas
        auto doc = raw_text_->document();
        while (doc->blockCount() > 50) {
            QTextCursor cursor(doc->begin());
            cursor.select(QTextCursor::BlockUnderCursor);
            cursor.removeSelectedText();
            cursor.deleteChar();  // salto de línea
        }
        last_raw_.clear();
    }

    if (!last_imu_.has_value()) {
        status_label_->setText("● Esperando datos...");
        return;
    }

    const auto& imu = *last_imu_;

    // Panel Parsed Fields
    auto setCell = [](QTableWidget* t, int row, const QString& val) {
        t->item(row, 1)->setText(val);
    };
    auto fmt = [](double v) { return QString::number(v, 'f', 6); };

    setCell(parsed_table_, 0,  fmt(imu.linear_acceleration.x));
    setCell(parsed_table_, 1,  fmt(imu.linear_acceleration.y));
    setCell(parsed_table_, 2,  fmt(imu.linear_acceleration.z));
    setCell(parsed_table_, 3,  fmt(imu.angular_velocity.x));
    setCell(parsed_table_, 4,  fmt(imu.angular_velocity.y));
    setCell(parsed_table_, 5,  fmt(imu.angular_velocity.z));
    setCell(parsed_table_, 6,  fmt(imu.orientation.x));
    setCell(parsed_table_, 7,  fmt(imu.orientation.y));
    setCell(parsed_table_, 8,  fmt(imu.orientation.z));
    setCell(parsed_table_, 9,  fmt(imu.orientation.w));
    setCell(parsed_table_, 10, QString::fromStdString(imu.header.frame_id));

    // Panel DDS Output
    setCell(dds_table_, 0,  "/imu/data");
    setCell(dds_table_, 1,  QString::number(msg_count_));
    setCell(dds_table_, 2,  QString::number(imu.header.stamp.sec));
    setCell(dds_table_, 3,  QString::number(imu.header.stamp.nanosec));
    setCell(dds_table_, 4,  QString::fromStdString(imu.header.frame_id));
    setCell(dds_table_, 5,  fmt(imu.orientation.x));
    setCell(dds_table_, 6,  fmt(imu.orientation.y));
    setCell(dds_table_, 7,  fmt(imu.orientation.z));
    setCell(dds_table_, 8,  fmt(imu.orientation.w));
    setCell(dds_table_, 9,  fmt(imu.linear_acceleration.z));
    setCell(dds_table_, 10, fmt(imu.angular_velocity.x));
    setCell(dds_table_, 11, "~50 Hz");

    status_label_->setText(
        QString("● Conectado  /dev/ttyUSB0  115200  50Hz  — msg #%1").arg(msg_count_));
}

} // namespace imu_dds_monitor
```

- [ ] **Step 5: Crear `src/imu_monitor_plugin.cpp`**

```cpp
// src/imu_monitor_plugin.cpp
#include "imu_dds_monitor/imu_monitor_widget.hpp"
#include <rqt_gui_cpp/plugin.h>
#include <pluginlib/class_list_macros.hpp>
#include <qt_gui_cpp/plugin_context.h>

namespace imu_dds_monitor {

class ImuMonitorPlugin : public rqt_gui_cpp::Plugin {
    Q_OBJECT
public:
    ImuMonitorPlugin() : rqt_gui_cpp::Plugin(), widget_(nullptr) {}

    void initPlugin(qt_gui_cpp::PluginContext& context) override {
        widget_ = new ImuMonitorWidget(node_);
        context.addWidget(widget_);
    }

    void shutdownPlugin() override {
        if (widget_) widget_->shutdown();
    }

    void saveSettings(qt_gui_cpp::Settings&, qt_gui_cpp::Settings&) const override {}
    void restoreSettings(const qt_gui_cpp::Settings&, const qt_gui_cpp::Settings&) override {}

private:
    ImuMonitorWidget* widget_{nullptr};
};

} // namespace imu_dds_monitor

#include "imu_monitor_plugin.moc"
PLUGINLIB_EXPORT_CLASS(imu_dds_monitor::ImuMonitorPlugin, rqt_gui_cpp::Plugin)
```

- [ ] **Step 6: Build el monitor**

```bash
cd ~/ros2_ws
colcon build --packages-select imu_dds_monitor 2>&1 | grep -E "Finished|error:"
```

Expected: `Finished <<< imu_dds_monitor`

- [ ] **Step 7: Verificar registro del plugin**

```bash
source ~/ros2_ws/install/setup.bash
rqt --list-plugins 2>&1 | grep -i imu
```

Expected: `imu_dds_monitor/ImuMonitorPlugin`

- [ ] **Step 8: Lanzar rqt con el plugin**

```bash
# Terminal 1: adaptador corriendo
# Terminal 2:
source ~/ros2_ws/install/setup.bash
rqt --standalone imu_dds_monitor/ImuMonitorPlugin
```

Expected: ventana con 3 tabs (Raw ASCII, Parsed Fields, DDS Output) con datos en vivo.

- [ ] **Step 9: Commit**

```bash
cd ~/ros2_ws/src
git add imu_dds_monitor/
git commit -m "feat: plugin rqt ImuMonitorPlugin — 3 tabs Raw/Parsed/DDS con datos en vivo"
```

---

## Task 9: RViz2 y Verificación del Sistema Completo

**Files:**
- Create: `imu_dds_adapter/config/imu_rviz.rviz`

- [ ] **Step 1: Instalar rviz_imu_plugin**

```bash
sudo apt-get install -y ros-lyrical-imu-tools
```

Expected: paquete instalado sin errores.

- [ ] **Step 2: Crear `config/imu_rviz.rviz`**

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_common/Grid
      Name: Grid
      Enabled: true
    - Class: rviz_imu_plugin/Imu
      Name: IMU
      Enabled: true
      Topic:
        Value: /imu/data
        Depth: 5
        Reliability Policy: Best Effort
        Durability Policy: Volatile
      Box enabled: true
      Axes enabled: true
      Box scale: 0.1
      Axes scale: 0.5
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: imu_link
    Frame Rate: 30
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.0
```

- [ ] **Step 3: Agregar el archivo .rviz al CMakeLists.txt de imu_dds_adapter**

En `imu_dds_adapter/CMakeLists.txt`, dentro del bloque `install`, agregar:

```cmake
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
```

(ya está en Task 1 — verificar que está presente)

- [ ] **Step 4: Lanzar el sistema completo**

```bash
# Terminal 1: adaptador
source ~/ros2_ws/install/setup.bash
ros2 launch imu_dds_adapter imu_dds.launch.py

# Terminal 2: RViz2
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/install/imu_dds_adapter/share/imu_dds_adapter/config/imu_rviz.rviz

# Terminal 3: plugin rqt
source ~/ros2_ws/install/setup.bash
rqt --standalone imu_dds_monitor/ImuMonitorPlugin
```

- [ ] **Step 5: Verificar criterios de éxito**

```bash
# Frecuencia 50 Hz
ros2 topic hz /imu/data
# Expected: average rate: 49.xxx

# Diagnóstico OK
ros2 topic echo /imu/status --once
# Expected: level: 0, name: VN-100 IMU Adapter

# Quaternion válido (norma ≈ 1.0)
ros2 topic echo /imu/data --once | grep -A4 orientation
# Expected: x, y, z, w con ‖q‖ ≈ 1.0

# Gravedad en Z
ros2 topic echo /imu/data --once | grep -A2 linear_acceleration
# Expected: z: -9.8...
```

- [ ] **Step 6: Verificar visualización en RViz2**

En RViz2: el cubo IMU debe rotar en tiempo real cuando se mueve el sensor físicamente. La orientación debe ser coherente con el movimiento real del sensor.

- [ ] **Step 7: Commit final**

```bash
cd ~/ros2_ws/src
colcon build --packages-select imu_dds_adapter imu_dds_monitor
git add imu_dds_adapter/config/imu_rviz.rviz
git commit -m "feat: configuracion RViz2 y verificacion del sistema completo — todos los criterios OK"
```

---

## Auto-revisión del Plan

### Cobertura del Spec

| Requisito del Spec | Tarea que lo implementa |
|---|---|
| Serial Driver — open, configureSensor, readline | Task 4 |
| NmeaParser — validateChecksum, parseVnimu, parseQuaternion | Task 2 |
| ImuConverter — NED→ENU, covariance, sensor_msgs/Imu | Task 3 |
| ImuPublisherNode — threading, queue, timer, QoS | Task 5 |
| Topics /imu/data, /imu/data_raw, /imu/raw_ascii, /imu/status | Task 5 |
| Plugin rqt — 3 tabs Raw/Parsed/DDS | Task 8 |
| Reconexión serial (3 reintentos) | Task 5 (tryReconfigure) |
| Polling cuaternión cada N ciclos | Task 5 (pollQuaternion) |
| Transform estático map→imu_link | Task 5 (publishStaticTransform) |
| RViz2 con rviz_imu_plugin | Task 9 |
| Diagnóstico /imu/status a 1 Hz | Task 5 (publishDiagnostics) |
| Parámetros en YAML | Task 6 |
| Criterios de éxito verificados | Task 7, Task 9 |

### Sin placeholders — todos los steps tienen código completo.
### Tipos consistentes — ImuFrame, NmeaParser, ImuConverter usados uniformemente en Tasks 2-5.
