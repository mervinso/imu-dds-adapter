#include "imu_dds_adapter/imu_converter.hpp"
#include <tf2/LinearMath/Quaternion.hpp>
#include <cmath>

namespace imu_dds_adapter {

sensor_msgs::msg::Imu ImuConverter::convert(
    const ImuFrame& frame,
    const std::string& frame_id)
{
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = frame_id;
    // stamp se asigna en ImuPublisherNode con rclcpp::Clock

    msg.linear_acceleration.x = frame.accel_x;
    msg.linear_acceleration.y = frame.accel_y;
    msg.linear_acceleration.z = frame.accel_z;
    msg.linear_acceleration_covariance = {
        kAccelVar, 0, 0,
        0, kAccelVar, 0,
        0, 0, kAccelVar
    };

    msg.angular_velocity.x = frame.gyro_x;
    msg.angular_velocity.y = frame.gyro_y;
    msg.angular_velocity.z = frame.gyro_z;
    msg.angular_velocity_covariance = {
        kGyroVar, 0, 0,
        0, kGyroVar, 0,
        0, 0, kGyroVar
    };

    if (!frame.quat_valid) {
        msg.orientation_covariance[0] = -1.0;
    } else {
        // NED→ENU: rotación 180° alrededor del eje (1/√2, 1/√2, 0)
        static const tf2::Quaternion kNedToEnu(
            1.0 / std::sqrt(2.0),  // x
            1.0 / std::sqrt(2.0),  // y
            0.0,                   // z
            0.0                    // w
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
