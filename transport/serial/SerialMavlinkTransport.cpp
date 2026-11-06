
#include "SerialMavlinkTransport.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

speed_t toTermiosSpeed(int baudRate)
{
    switch (baudRate) {
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 921600:
        return B921600;
    default:
        return 0;
    }
}

} // namespace


SerialMavlinkTransport::~SerialMavlinkTransport()
{
    if (fd_ >= 0) {
        close(fd_);
    }
}


bool SerialMavlinkTransport::openPort(
    const std::string& device,
    int baudRate
)
{
    if (fd_ >= 0) {
        std::cerr
            << "[SERIAL] Already open\n";
        return false;
    }

    const speed_t speed =
        toTermiosSpeed(baudRate);

    if (speed == 0) {
        std::cerr
            << "[SERIAL] Unsupported baud rate\n";
        return false;
    }

    fd_ = open(
        device.c_str(),
        O_RDONLY | O_NOCTTY | O_NONBLOCK
    );

    if (fd_ < 0) {
        std::cerr
            << "[SERIAL] Open failed: "
            << std::strerror(errno)
            << "\n";
        return false;
    }

    termios tty{};

    if (tcgetattr(fd_, &tty) != 0) {
        perror("[SERIAL] tcgetattr");
        close(fd_);
        fd_ = -1;
        return false;
    }

    cfmakeraw(&tty);

    tty.c_cflag |=
        (CLOCAL | CREAD);

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (
        cfsetispeed(&tty, speed) != 0 ||
        cfsetospeed(&tty, speed) != 0
    ) {
        perror("[SERIAL] baud rate");
        close(fd_);
        fd_ = -1;
        return false;
    }

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (
        tcsetattr(
            fd_,
            TCSANOW,
            &tty
        ) != 0
    ) {
        perror("[SERIAL] tcsetattr");
        close(fd_);
        fd_ = -1;
        return false;
    }

    std::cout
        << "[SERIAL] Opened "
        << device
        << " @ "
        << baudRate
        << " baud\n";

    return true;
}


ssize_t SerialMavlinkTransport::receive(
    uint8_t* buffer,
    std::size_t capacity
)
{
    if (
        fd_ < 0 ||
        buffer == nullptr ||
        capacity == 0
    ) {
        errno = EINVAL;
        return -1;
    }

    return read(
        fd_,
        buffer,
        capacity
    );
}


bool SerialMavlinkTransport::isOpen() const
{
    return fd_ >= 0;
}
