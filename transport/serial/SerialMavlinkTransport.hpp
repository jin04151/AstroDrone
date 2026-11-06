
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/types.h>

class SerialMavlinkTransport {
public:
    SerialMavlinkTransport() = default;
    ~SerialMavlinkTransport();

    SerialMavlinkTransport(
        const SerialMavlinkTransport&
    ) = delete;

    SerialMavlinkTransport& operator=(
        const SerialMavlinkTransport&
    ) = delete;

    bool openPort(
        const std::string& device,
        int baudRate
    );

    // Read-only UART transport.
    // No flight command transmission API.
    ssize_t receive(
        uint8_t* buffer,
        std::size_t capacity
    );

    bool isOpen() const;

private:
    int fd_ = -1;
};
