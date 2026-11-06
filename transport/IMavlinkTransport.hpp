
#pragma once

#include <cstddef>
#include <cstdint>
#include <sys/types.h>

// Common interface for MAVLink byte transport.
//
// UDP : receive + transmit
// UART: receive only (for current REAL app)
//
// This interface transports raw bytes.
// MAVLink parsing is handled separately.
class IMavlinkTransport {
public:
    virtual ~IMavlinkTransport() = default;

    // Read incoming bytes without blocking.
    virtual ssize_t receive(
        uint8_t* buffer,
        std::size_t capacity
    ) = 0;

    virtual bool isOpen() const = 0;

    // Transmit bytes.
    // Default: transmission forbidden.
    virtual bool send(
        const uint8_t* buffer,
        std::size_t length
    ) {
        (void)buffer;
        (void)length;
        return false;
    }

    // UDP peer handling.
    // Serial transport does not need these.
    virtual void confirmLastSender() {
    }

    virtual bool hasPeer() const {
        return false;
    }
};
