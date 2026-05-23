#include "imu_dds_monitor/imu_monitor_widget.hpp"
#include <rqt_gui_cpp/plugin.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <qt_gui_cpp/plugin_context.hpp>

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
