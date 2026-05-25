cmake_minimum_required(VERSION 3.8)
project(imu_dds_spy)

set(CMAKE_CXX_STANDARD 17)

set(FASTDDS_ROOT /opt/ros/lyrical)

add_executable(imu_dds_spy imu_dds_spy.cpp)

target_include_directories(imu_dds_spy PRIVATE
    ${FASTDDS_ROOT}/include/fastdds
    ${FASTDDS_ROOT}/include
    ${FASTDDS_ROOT}/includefastcdr
)

target_link_directories(imu_dds_spy PRIVATE
    ${FASTDDS_ROOT}/lib
    ${FASTDDS_ROOT}/lib/x86_64-linux-gnu
)

target_link_libraries(imu_dds_spy fastdds fastcdr)

set_target_properties(imu_dds_spy PROPERTIES
    BUILD_RPATH "${FASTDDS_ROOT}/lib;${FASTDDS_ROOT}/lib/x86_64-linux-gnu"
)
