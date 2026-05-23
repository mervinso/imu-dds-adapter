#pragma once
#include <string>
#include <stdexcept>
#include <termios.h>

namespace imu_dds_adapter {

class SerialPortException : public std::runtime_error {
public:
    explicit SerialPortException(const std::string& msg) : std::runtime_error(msg) {}
};

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    // Abre el puerto y configura baud rate.
    // Lanza SerialPortException si falla.
    void open(const std::string& device, int baud_rate);

    // Envía la secuencia de configuración al VN-100:
    //   $VNASY,0    — pausa stream
    //   $VNWRG,6,19 — output type = VNIMU (Registro 54)
    //   $VNWRG,7,<hz> — frecuencia
    //   $VNASY,1    — reanuda stream
    void configureSensor(int output_hz);

    // Lee una línea completa terminada en \n. Bloqueante.
    // Lanza SerialPortException si timeout (500ms sin datos).
    std::string readline();

    // Escribe un comando ASCII al sensor. Agrega \r\n automáticamente.
    void write(const std::string& cmd);

    void close();
    bool isOpen() const { return fd_ >= 0; }

private:
    int fd_{-1};

    // Convierte baud rate numérico a constante termios (B115200, etc.)
    static speed_t toBaudRate(int baud);
};

} // namespace imu_dds_adapter
