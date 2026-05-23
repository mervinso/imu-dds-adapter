#pragma once
#include "imu_dds_adapter/imu_frame.hpp"
#include <optional>
#include <string>
#include <vector>

namespace imu_dds_adapter {

class NmeaParser {
public:
    // Valida checksum XOR 8-bit. Formato: $PAYLOAD*HH
    static bool validateChecksum(const std::string& line);

    // Parsea mensaje $VNIMU (Registro 54 async output).
    // Retorna nullopt si checksum inválido, header incorrecto o campos insuficientes.
    static std::optional<ImuFrame> parseVnimu(const std::string& line);

    // Parsea respuesta $VNRRG,09 (quaternion polling).
    // Escribe qx,qy,qz,qw en frame y setea quat_valid=true.
    // Retorna false si el mensaje no es VNRRG,09 o tiene checksum inválido.
    static bool parseQuaternion(const std::string& line, ImuFrame& frame);

private:
    // Divide línea en tokens por delimitador. Excluye el prefijo $ y el *CS final.
    static std::vector<std::string> tokenize(const std::string& line);
};

} // namespace imu_dds_adapter
