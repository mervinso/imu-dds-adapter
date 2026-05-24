cmake_minimum_required(VERSION 3.8)
project(imu_dds_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(ADAPTER_SOURCES
  src/nmea_parser.cpp
  src/imu_converter.cpp
  src/serial_port.cpp
  src/imu_publisher_node.cpp
)

add_executable(imu_dds_adapter_node src/main.cpp ${ADAPTER_SOURCES})

target_include_directories(imu_dds_adapter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(imu_dds_adapter_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  diagnostic_msgs::diagnostic_msgs
  geometry_msgs::geometry_msgs
  tf2::tf2
  tf2_ros::tf2_ros
)

install(TARGETS imu_dds_adapter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY urdf/    DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_nmea_parser
    test/test_nmea_parser.cpp
    src/nmea_parser.cpp
  )
  target_include_directories(test_nmea_parser PRIVATE include)

  ament_add_gtest(test_imu_converter
    test/test_imu_converter.cpp
    src/imu_converter.cpp
  )
  target_include_directories(test_imu_converter PRIVATE include)
  target_link_libraries(test_imu_converter
    sensor_msgs::sensor_msgs
    geometry_msgs::geometry_msgs
    tf2::tf2
  )
endif()

ament_package()
