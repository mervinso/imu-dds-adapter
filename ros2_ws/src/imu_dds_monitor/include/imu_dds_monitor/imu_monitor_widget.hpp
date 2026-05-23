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
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr  imu_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  raw_sub_;

    std::mutex data_mutex_;
    std::optional<sensor_msgs::msg::Imu> last_imu_;
    std::string last_raw_;
    uint64_t msg_count_{0};

    QTimer* refresh_timer_;

    QTabWidget*   tab_widget_;
    QTextEdit*    raw_text_;
    QTableWidget* parsed_table_;
    QTableWidget* dds_table_;
    QLabel*       status_label_;

    Ui::ImuMonitorWidget* ui_;
};

} // namespace imu_dds_monitor
