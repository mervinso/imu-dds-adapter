#include <gtest/gtest.h>
#include "imu_dds_adapter/nmea_parser.hpp"
#include "imu_dds_adapter/imu_frame.hpp"
#include <cstdint>
#include <cstdio>
#include <string>

using namespace imu_dds_adapter;

static std::string makeMsg(const std::string& payload) {
    uint8_t cs = 0;
    for (char c : payload) cs ^= static_cast<uint8_t>(c);
    char buf[3];
    snprintf(buf, sizeof(buf), "%02X", cs);
    return "$" + payload + "*" + buf;
}

TEST(NmeaParser, ValidChecksumPasses) {
    EXPECT_TRUE(NmeaParser::validateChecksum("$VNYPR,-129.867,-000.355,-000.296*61"));
}

TEST(NmeaParser, InvalidChecksumFails) {
    EXPECT_FALSE(NmeaParser::validateChecksum("$VNYPR,-129.867,-000.355,-000.296*62"));
}

TEST(NmeaParser, MissingDollarFails) {
    EXPECT_FALSE(NmeaParser::validateChecksum("VNYPR,-129.867*61"));
}

TEST(NmeaParser, MissingStarFails) {
    EXPECT_FALSE(NmeaParser::validateChecksum("$VNYPR,-129.867"));
}

TEST(NmeaParser, ParsesVnimuFields) {
    std::string payload =
        "VNIMU,-00.1462,+00.1761,+00.2623,"
        "-00.0780,+00.0700,-09.8010,"
        "+00.007910,-00.003443,-00.009322,"
        "+36.3,+100.871";
    std::string line = makeMsg(payload);

    auto result = NmeaParser::parseVnimu(line);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->mag_x,       -0.1462,   1e-4);
    EXPECT_NEAR(result->mag_y,       +0.1761,   1e-4);
    EXPECT_NEAR(result->mag_z,       +0.2623,   1e-4);
    EXPECT_NEAR(result->accel_x,     -0.0780,   1e-4);
    EXPECT_NEAR(result->accel_y,     +0.0700,   1e-4);
    EXPECT_NEAR(result->accel_z,     -9.8010,   1e-3);
    EXPECT_NEAR(result->gyro_x,      +0.007910, 1e-6);
    EXPECT_NEAR(result->gyro_y,      -0.003443, 1e-6);
    EXPECT_NEAR(result->gyro_z,      -0.009322, 1e-6);
    EXPECT_NEAR(result->temperature, 36.3,      0.05);
    EXPECT_NEAR(result->pressure,    100.871,   1e-3);
}

TEST(NmeaParser, ParseVnimuRejectsWrongHeader) {
    std::string payload = "VNYPR,-129.867,-000.355,-000.296";
    std::string line = makeMsg(payload);
    EXPECT_FALSE(NmeaParser::parseVnimu(line).has_value());
}

TEST(NmeaParser, ParseVnimuRejectsInvalidChecksum) {
    std::string line = "$VNIMU,-00.1462,+00.1761,+00.2623,"
                       "-00.0780,+00.0700,-09.8010,"
                       "+00.007910,-00.003443,-00.009322,"
                       "+36.3,+100.871*FF";
    EXPECT_FALSE(NmeaParser::parseVnimu(line).has_value());
}

TEST(NmeaParser, ParseVnimuRejectsTooFewFields) {
    std::string payload = "VNIMU,-00.1462,+00.1761";
    std::string line = makeMsg(payload);
    EXPECT_FALSE(NmeaParser::parseVnimu(line).has_value());
}

TEST(NmeaParser, ParsesQuaternionFromVnrrg09) {
    std::string payload = "VNRRG,09,+0.004846,+0.000221,+0.905755,-0.423774";
    std::string line = makeMsg(payload);

    ImuFrame frame;
    EXPECT_TRUE(NmeaParser::parseQuaternion(line, frame));
    EXPECT_NEAR(frame.qx, +0.004846,  1e-6);
    EXPECT_NEAR(frame.qy, +0.000221,  1e-6);
    EXPECT_NEAR(frame.qz, +0.905755,  1e-6);
    EXPECT_NEAR(frame.qw, -0.423774,  1e-6);
    EXPECT_TRUE(frame.quat_valid);
}

TEST(NmeaParser, ParseQuaternionRejectsWrongHeader) {
    std::string payload = "VNRRG,08,-129.847,-000.510,-000.211";
    std::string line = makeMsg(payload);
    ImuFrame frame;
    EXPECT_FALSE(NmeaParser::parseQuaternion(line, frame));
    EXPECT_FALSE(frame.quat_valid);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
