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
