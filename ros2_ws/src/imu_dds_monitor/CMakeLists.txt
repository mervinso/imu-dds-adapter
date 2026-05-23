cmake_minimum_required(VERSION 3.8)
project(imu_dds_monitor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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

target_link_libraries(imu_dds_monitor
  ${QT_LIBRARIES}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  rqt_gui_cpp::rqt_gui_cpp
  qt_gui_cpp::qt_gui_cpp
  pluginlib::pluginlib
)

install(TARGETS imu_dds_monitor DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(rqt_gui plugin.xml)

ament_package()
