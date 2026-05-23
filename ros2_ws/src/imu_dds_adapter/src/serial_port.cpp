#include "imu_dds_adapter/serial_port.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <cerrno>
#include <cstring>
#include <sstream>
#include <thread>
#include <chrono>

namespace imu_dds_adapter {

SerialPort::~SerialPort() { close(); }

speed_t SerialPort::toBaudRate(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 921600: return B921600;
        default:
            throw SerialPortException("Baud rate no soportado: " + std::to_string(baud));
    }
}

void SerialPort::open(const std::string& device, int baud_rate) {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        throw SerialPortException("No se pudo abrir " + device + ": " + strerror(errno));
    }

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        ::close(fd_); fd_ = -1;
        throw SerialPortException("tcgetattr falló: " + std::string(strerror(errno)));
    }

    speed_t speed = toBaudRate(baud_rate);
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP);
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;  // timeout 0.5s por carácter

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        ::close(fd_); fd_ = -1;
        throw SerialPortException("tcsetattr falló: " + std::string(strerror(errno)));
    }
    tcflush(fd_, TCIFLUSH);
}

void SerialPort::write(const std::string& cmd) {
    std::string full = cmd + "\r\n";
    ssize_t written = ::write(fd_, full.c_str(), full.size());
    if (written < 0) {
        throw SerialPortException("Error escribiendo al puerto: " + std::string(strerror(errno)));
    }
}

void SerialPort::configureSensor(int output_hz) {
    using namespace std::chrono_literals;

    write("$VNASY,0*XX");
    std::this_thread::sleep_for(200ms);

    write("$VNWRG,6,19*XX");
    std::this_thread::sleep_for(100ms);

    write("$VNWRG,7," + std::to_string(output_hz) + "*XX");
    std::this_thread::sleep_for(100ms);

    write("$VNASY,1*XX");
    std::this_thread::sleep_for(200ms);

    tcflush(fd_, TCIFLUSH);
}

std::string SerialPort::readline() {
    std::string line;
    char ch;

    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;

    while (true) {
        int ret = poll(&pfd, 1, 500);
        if (ret < 0) {
            throw SerialPortException("poll falló: " + std::string(strerror(errno)));
        }
        if (ret == 0) {
            throw SerialPortException("Timeout leyendo del puerto serial");
        }

        ssize_t n = ::read(fd_, &ch, 1);
        if (n < 0) {
            throw SerialPortException("Error leyendo del puerto: " + std::string(strerror(errno)));
        }
        if (n == 0) continue;

        if (ch == '\n') return line;
        if (ch != '\r') line += ch;
    }
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

} // namespace imu_dds_adapter
