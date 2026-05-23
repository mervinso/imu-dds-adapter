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
