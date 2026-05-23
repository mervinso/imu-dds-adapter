#include "imu_dds_adapter/nmea_parser.hpp"
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <vector>

namespace imu_dds_adapter {

bool NmeaParser::validateChecksum(const std::string& line) {
    if (line.empty() || line.front() != '$') return false;
    auto star = line.rfind('*');
    if (star == std::string::npos || star + 3 > line.size()) return false;

    uint8_t computed = 0;
    for (size_t i = 1; i < star; ++i) {
        computed ^= static_cast<uint8_t>(line[i]);
    }

    try {
        uint8_t expected = static_cast<uint8_t>(
            std::stoul(line.substr(star + 1, 2), nullptr, 16));
        return computed == expected;
    } catch (...) {
        return false;
    }
}

std::vector<std::string> NmeaParser::tokenize(const std::string& line) {
    auto start = line.find('$');
    auto end   = line.rfind('*');
    if (start == std::string::npos || end == std::string::npos) return {};

    std::string payload = line.substr(start + 1, end - start - 1);
    std::vector<std::string> tokens;
    std::stringstream ss(payload);
    std::string token;
    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }
    return tokens;
}

std::optional<ImuFrame> NmeaParser::parseVnimu(const std::string& line) {
    if (!validateChecksum(line)) return std::nullopt;

    auto tokens = tokenize(line);
    // tokens[0]="VNIMU", tokens[1..11]=11 campos de datos
    if (tokens.size() < 12 || tokens[0] != "VNIMU") return std::nullopt;

    try {
        ImuFrame f;
        f.mag_x       = std::stod(tokens[1]);
        f.mag_y       = std::stod(tokens[2]);
        f.mag_z       = std::stod(tokens[3]);
        f.accel_x     = std::stod(tokens[4]);
        f.accel_y     = std::stod(tokens[5]);
        f.accel_z     = std::stod(tokens[6]);
        f.gyro_x      = std::stod(tokens[7]);
        f.gyro_y      = std::stod(tokens[8]);
        f.gyro_z      = std::stod(tokens[9]);
        f.temperature = std::stod(tokens[10]);
        f.pressure    = std::stod(tokens[11]);
        return f;
    } catch (...) {
        return std::nullopt;
    }
}

bool NmeaParser::parseQuaternion(const std::string& line, ImuFrame& frame) {
    if (!validateChecksum(line)) return false;

    auto tokens = tokenize(line);
    // tokens[0]="VNRRG", tokens[1]="09", tokens[2..5]=qx,qy,qz,qw
    if (tokens.size() < 6 || tokens[0] != "VNRRG" || tokens[1] != "09") return false;

    try {
        frame.qx         = std::stod(tokens[2]);
        frame.qy         = std::stod(tokens[3]);
        frame.qz         = std::stod(tokens[4]);
        frame.qw         = std::stod(tokens[5]);
        frame.quat_valid = true;
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace imu_dds_adapter
