
#pragma once

#include <cstddef>
#include <cstdint>

#include <netinet/in.h>
#include <sys/types.h>

#include "IMavlinkTransport.hpp"

class UdpMavlinkTransport final
    : public IMavlinkTransport {
public:
    UdpMavlinkTransport() = default;
    ~UdpMavlinkTransport() override;

    UdpMavlinkTransport(
        const UdpMavlinkTransport&
    ) = delete;

    UdpMavlinkTransport& operator=(
        const UdpMavlinkTransport&
    ) = delete;

    bool openPort(int port);

    ssize_t receive(
        uint8_t* buffer,
        std::size_t capacity
    ) override;

    void confirmLastSender() override;

    bool send(
        const uint8_t* buffer,
        std::size_t length
    ) override;

    bool isOpen() const override;
    bool hasPeer() const override;

private:
    int socket_ = -1;

    sockaddr_in last_sender_{};
    bool last_sender_valid_ = false;

    sockaddr_in peer_address_{};
    bool peer_valid_ = false;
};
