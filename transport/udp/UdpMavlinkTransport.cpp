
#include "UdpMavlinkTransport.hpp"

#include <cerrno>
#include <iostream>

#include <sys/socket.h>
#include <unistd.h>

UdpMavlinkTransport::~UdpMavlinkTransport()
{
    if (socket_ >= 0) {
        close(socket_);
    }
}


bool UdpMavlinkTransport::openPort(int port)
{
    if (socket_ >= 0) {
        std::cerr
            << "[UDP] Socket already open\n";
        return false;
    }

    if (port <= 0 || port > 65535) {
        std::cerr
            << "[UDP] Invalid port\n";
        return false;
    }

    socket_ = socket(
        AF_INET,
        SOCK_DGRAM,
        0
    );

    if (socket_ < 0) {
        perror("[UDP] socket");
        return false;
    }

    sockaddr_in address{};

    address.sin_family = AF_INET;
    address.sin_port = htons(
        static_cast<uint16_t>(port)
    );

    address.sin_addr.s_addr =
        htonl(INADDR_ANY);

    if (
        bind(
            socket_,
            reinterpret_cast<sockaddr*>(
                &address
            ),
            sizeof(address)
        ) < 0
    ) {
        perror("[UDP] bind");

        close(socket_);
        socket_ = -1;

        return false;
    }

    last_sender_valid_ = false;
    peer_valid_ = false;

    std::cout
        << "[MAVLINK] Listening UDP "
        << port
        << "\n";

    return true;
}


ssize_t UdpMavlinkTransport::receive(
    uint8_t* buffer,
    std::size_t capacity
)
{
    if (
        socket_ < 0 ||
        buffer == nullptr ||
        capacity == 0
    ) {
        errno = EINVAL;
        return -1;
    }

    sockaddr_in sender{};
    socklen_t sender_length =
        sizeof(sender);

    const ssize_t count = recvfrom(
        socket_,
        buffer,
        capacity,
        MSG_DONTWAIT,
        reinterpret_cast<sockaddr*>(
            &sender
        ),
        &sender_length
    );

    if (count > 0) {
        last_sender_ = sender;
        last_sender_valid_ = true;
    } else {
        last_sender_valid_ = false;
    }

    return count;
}


void UdpMavlinkTransport::confirmLastSender()
{
    if (!last_sender_valid_)
        return;

    peer_address_ = last_sender_;
    peer_valid_ = true;
}


bool UdpMavlinkTransport::send(
    const uint8_t* buffer,
    std::size_t length
)
{
    if (
        socket_ < 0 ||
        !peer_valid_ ||
        buffer == nullptr ||
        length == 0
    ) {
        return false;
    }

    const ssize_t sent = sendto(
        socket_,
        buffer,
        length,
        0,
        reinterpret_cast<const sockaddr*>(
            &peer_address_
        ),
        sizeof(peer_address_)
    );

    if (
        sent != static_cast<ssize_t>(
            length
        )
    ) {
        perror("[UDP] sendto");
        return false;
    }

    return true;
}


bool UdpMavlinkTransport::isOpen() const
{
    return socket_ >= 0;
}


bool UdpMavlinkTransport::hasPeer() const
{
    return peer_valid_;
}
