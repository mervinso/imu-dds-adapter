#include <gtest/gtest.h>
#include "imu_dds_adapter/imu_converter.hpp"
#include "imu_dds_adapter/imu_frame.hpp"
#include <cmath>

using namespace imu_dds_adapter;

static ImuFrame makeFrame() {
    ImuFrame f;
    f.accel_x = -0.078; f.accel_y = 0.070; f.accel_z = -9.801;
    f.gyro_x  =  0.0079; f.gyro_y = -0.0034; f.gyro_z = -0.0093;
    f.mag_x   = -0.1462; f.mag_y  =  0.1761; f.mag_z  =  0.2623;
    f.temperature = 36.3; f.pressure = 100.871;
    f.qx = 0.004846; f.qy = 0.000221; f.qz = 0.905755; f.qw = -0.423774;
    f.quat_valid = true;
    return f;
}

TEST(ImuConverter, LinearAccelerationMapsDirectly) {
    auto msg = ImuConverter::convert(makeFrame(), "imu_link");
    EXPECT_NEAR(msg.linear_acceleration.x, -0.078,  1e-4);
    EXPECT_NEAR(msg.linear_acceleration.y,  0.070,  1e-4);
    EXPECT_NEAR(msg.linear_acceleration.z, -9.801,  1e-3);
}

TEST(ImuConverter, AngularVelocityMapsDirectly) {
    auto msg = ImuConverter::convert(makeFrame(), "imu_link");
    EXPECT_NEAR(msg.angular_velocity.x,  0.0079, 1e-5);
    EXPECT_NEAR(msg.angular_velocity.y, -0.0034, 1e-5);
    EXPECT_NEAR(msg.angular_velocity.z, -0.0093, 1e-5);
}

TEST(ImuConverter, OrientationQuaternionIsUnitVector) {
    auto msg = ImuConverter::convert(makeFrame(), "imu_link");
    double norm = std::sqrt(
        msg.orientation.x * msg.orientation.x +
        msg.orientation.y * msg.orientation.y +
        msg.orientation.z * msg.orientation.z +
        msg.orientation.w * msg.orientation.w
    );
    EXPECT_NEAR(norm, 1.0, 1e-6);
}

TEST(ImuConverter, FrameIdIsSet) {
    auto msg = ImuConverter::convert(makeFrame(), "imu_link");
    EXPECT_EQ(msg.header.frame_id, "imu_link");
}

TEST(ImuConverter, CovarianceUnknownWhenQuatInvalid) {
    ImuFrame f = makeFrame();
    f.quat_valid = false;
    auto msg = ImuConverter::convert(f, "imu_link");
    EXPECT_DOUBLE_EQ(msg.orientation_covariance[0], -1.0);
}

TEST(ImuConverter, CovarianceDiagonalWhenQuatValid) {
    auto msg = ImuConverter::convert(makeFrame(), "imu_link");
    EXPECT_GT(msg.orientation_covariance[0], 0.0);
    EXPECT_DOUBLE_EQ(msg.orientation_covariance[1], 0.0);
    EXPECT_GT(msg.angular_velocity_covariance[0], 0.0);
    EXPECT_DOUBLE_EQ(msg.angular_velocity_covariance[1], 0.0);
    EXPECT_GT(msg.linear_acceleration_covariance[0], 0.0);
    EXPECT_DOUBLE_EQ(msg.linear_acceleration_covariance[1], 0.0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
