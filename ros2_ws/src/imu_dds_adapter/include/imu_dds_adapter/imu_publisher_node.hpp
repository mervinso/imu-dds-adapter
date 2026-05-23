#pragma once
#include "imu_dds_adapter/serial_port.hpp"
#include "imu_dds_adapter/nmea_parser.hpp"
#include "imu_dds_adapter/imu_converter.hpp"
#include "imu_dds_adapter/imu_frame.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
    void serialReaderThread();
    void timerCallback();
    void pollQuaternion();
    void publishDiagnostics();
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
    std::mutex  serial_mutex_;  // protege acceso concurrente al puerto

    // ── Threading ────────────────────────────────────────────────
    std::thread reader_thread_;
    std::atomic<bool> running_{false};

    std::queue<std::string> frame_queue_;
    std::mutex queue_mutex_;

    ImuFrame quat_cache_;
    std::mutex quat_mutex_;
    int quat_poll_counter_{0};

    // ── Publishers ───────────────────────────────────────────────
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                imu_raw_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                raw_ascii_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr         pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ── Métricas de diagnóstico ──────────────────────────────────
    std::atomic<uint64_t> frames_received_{0};
    std::atomic<uint64_t> frames_dropped_{0};
    std::atomic<uint64_t> checksum_errors_{0};
};

} // namespace imu_dds_adapter
