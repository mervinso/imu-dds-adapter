#include "imu_dds_monitor/imu_monitor_widget.hpp"
#include "ui_imu_monitor.h"

#include <QHeaderView>
#include <QTextBlock>
#include <QTextCursor>
#include <QTextDocument>
#include <cmath>
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

    refresh_timer_ = new QTimer(this);
    connect(refresh_timer_, &QTimer::timeout, this, &ImuMonitorWidget::refreshUi);
    refresh_timer_->start(100);
}

ImuMonitorWidget::~ImuMonitorWidget() { delete ui_; }

void ImuMonitorWidget::shutdown() {
    refresh_timer_->stop();
    imu_sub_.reset();
    raw_sub_.reset();
}

void ImuMonitorWidget::setupParsedTable() {
    parsed_table_->setHorizontalHeaderLabels({"Campo", "Valor"});
    QStringList fields = {
        "accel_x [m/s2]", "accel_y [m/s2]", "accel_z [m/s2]",
        "gyro_x [rad/s]", "gyro_y [rad/s]",  "gyro_z [rad/s]",
        "quat_x",         "quat_y",           "quat_z", "quat_w",
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
    dds_table_->setHorizontalHeaderLabels({"Campo DDS", "Valor"});
    QStringList fields = {
        "topic", "seq",
        "stamp.sec", "stamp.nanosec",
        "frame_id",
        "orient.x", "orient.y", "orient.z", "orient.w",
        "lin_acc.z [m/s2]",
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

    if (!last_raw_.empty()) {
        raw_text_->append(QString::fromStdString(last_raw_));
        auto doc = raw_text_->document();
        while (doc->blockCount() > 50) {
            QTextCursor cursor(doc->begin());
            cursor.select(QTextCursor::BlockUnderCursor);
            cursor.removeSelectedText();
            cursor.deleteChar();
        }
        last_raw_.clear();
    }

    if (!last_imu_.has_value()) {
        status_label_->setText(QString::fromUtf8("\xe2\x97\x8f Esperando datos..."));
        return;
    }

    const auto& imu = *last_imu_;

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
        QString::fromUtf8("\xe2\x97\x8f Conectado  /dev/ttyUSB0  115200  50Hz  \xe2\x80\x94 msg #%1")
            .arg(msg_count_));
}

} // namespace imu_dds_monitor
