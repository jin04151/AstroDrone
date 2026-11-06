
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <sys/types.h>

#include "IMavlinkTransport.hpp"

// Receive-only serial transport.
//
// Used for Pixhawk telemetry.
// No send() override is provided.
//
// IMavlinkTransport::send() always returns false.
class SerialMavlinkTransport final
    : public IMavlinkTransport {
public:
    SerialMavlinkTransport() = default;
    ~SerialMavlinkTransport() override;

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

    ssize_t receive(
        uint8_t* buffer,
        std::size_t capacity
    ) override;

    bool isOpen() const override;

private:
    int fd_ = -1;
};
