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
    static constexpr double kGyroVar   = 1.2e-7;  // rad²/s²
    static constexpr double kAccelVar  = 4.0e-5;  // m²/s⁴
    static constexpr double kOrientVar = 1.0e-4;  // rad²
};

} // namespace imu_dds_adapter
