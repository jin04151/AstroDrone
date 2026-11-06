
#pragma once

#include <cstddef>
#include <cstdint>

#include <netinet/in.h>
#include <sys/types.h>

class UdpMavlinkTransport {
public:
    UdpMavlinkTransport() = default;
    ~UdpMavlinkTransport();

    UdpMavlinkTransport(
        const UdpMavlinkTransport&
    ) = delete;

    UdpMavlinkTransport& operator=(
        const UdpMavlinkTransport&
    ) = delete;

    // Bind a local UDP receiving port.
    bool openPort(int port);

    // Receive one UDP datagram.
    // Returns -1 when no data is available.
    ssize_t receive(
        uint8_t* buffer,
        std::size_t capacity
    );

    // Trust the sender of the last datagram
    // only after MavlinkClient verifies
    // an autopilot HEARTBEAT.
    void confirmLastSender();

    // Send MAVLink bytes to the
    // confirmed peer address.
    bool send(
        const uint8_t* buffer,
        std::size_t length
    );

    bool isOpen() const;
    bool hasPeer() const;

private:
    int socket_ = -1;

    sockaddr_in last_sender_{};
    bool last_sender_valid_ = false;

    sockaddr_in peer_address_{};
    bool peer_valid_ = false;
};
