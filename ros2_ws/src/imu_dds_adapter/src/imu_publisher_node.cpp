#include "imu_dds_adapter/imu_publisher_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace imu_dds_adapter {

ImuPublisherNode::ImuPublisherNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("imu_dds_adapter", options)
{
    serial_port_        = declare_parameter<std::string>("serial_port",        "/dev/ttyUSB0");
    baud_rate_          = declare_parameter<int>        ("baud_rate",          115200);
    output_hz_          = declare_parameter<int>        ("output_hz",          50);
    quat_poll_every_n_  = declare_parameter<int>        ("quat_poll_every_n",  5);
    frame_id_           = declare_parameter<std::string>("frame_id",           "imu_link");
    reconfigure_sensor_ = declare_parameter<bool>       ("reconfigure_sensor", true);

    imu_pub_       = create_publisher<sensor_msgs::msg::Imu>("/imu/data",      10);
    imu_raw_pub_   = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw",  10);
    raw_ascii_pub_ = create_publisher<std_msgs::msg::String>("/imu/raw_ascii", 10);
    status_pub_    = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/imu/status", 1);

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

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

    running_ = true;
    reader_thread_ = std::thread(&ImuPublisherNode::serialReaderThread, this);

    auto period = std::chrono::duration<double>(1.0 / output_hz_);
    timer_ = create_wall_timer(period, [this]() { timerCallback(); });

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
            std::string line;
            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                line = serial_.readline();
            }

            auto raw_msg = std_msgs::msg::String();
            raw_msg.data = line;
            raw_ascii_pub_->publish(raw_msg);

            {
                std::lock_guard<std::mutex> lock(quat_mutex_);
                if (NmeaParser::parseQuaternion(line, quat_cache_)) continue;
            }

            if (!NmeaParser::validateChecksum(line)) {
                checksum_errors_++;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Checksum inválido: %s", line.c_str());
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (frame_queue_.size() > 10) {
                    frame_queue_.pop();
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

    {
        std::lock_guard<std::mutex> lock(quat_mutex_);
        frame.qx         = quat_cache_.qx;
        frame.qy         = quat_cache_.qy;
        frame.qz         = quat_cache_.qz;
        frame.qw         = quat_cache_.qw;
        frame.quat_valid = quat_cache_.quat_valid;
    }

    auto imu_msg = ImuConverter::convert(frame, frame_id_);
    imu_msg.header.stamp = now();
    imu_pub_->publish(imu_msg);
    imu_raw_pub_->publish(imu_msg);

    quat_poll_counter_++;
    if (quat_poll_counter_ >= quat_poll_every_n_) {
        quat_poll_counter_ = 0;
        pollQuaternion();
    }
}

void ImuPublisherNode::pollQuaternion() {
    try {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_.write("$VNRRG,09*XX");
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

    msg.values.push_back(kv("frames_received", frames_received_.load()));
    msg.values.push_back(kv("frames_dropped",  frames_dropped_.load()));
    msg.values.push_back(kv("checksum_errors", checksum_errors_.load()));

    status_pub_->publish(msg);
}

bool ImuPublisherNode::tryReconfigure() {
    for (int i = 0; i < 3; ++i) {
        RCLCPP_WARN(get_logger(), "Intento de reconfiguración %d/3", i + 1);
        try {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            serial_.configureSensor(output_hz_);
            return true;
        } catch (...) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    return false;
}

} // namespace imu_dds_adapter
