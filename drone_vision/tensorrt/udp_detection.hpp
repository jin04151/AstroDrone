#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>

class UdpDetectionSender {
public:
    UdpDetectionSender(const std::string& host, int port) {
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) throw std::runtime_error("UDP socket: " + std::string(std::strerror(errno)));
        address_.sin_family = AF_INET;
        address_.sin_port = htons(static_cast<uint16_t>(port));
        if (inet_pton(AF_INET, host.c_str(), &address_.sin_addr) != 1) {
            close(fd_);
            throw std::runtime_error("Invalid numeric IPv4 UDP host");
        }
    }
    ~UdpDetectionSender() { if (fd_ >= 0) close(fd_); }
    UdpDetectionSender(const UdpDetectionSender&) = delete;
    UdpDetectionSender& operator=(const UdpDetectionSender&) = delete;

    bool send(bool detected, int class_id, float confidence, int cx, int cy, int width, int height) const {
        // VisionReceiver protocol: detected,class_id,confidence,cx,cy,width,height
        std::ostringstream msg;
        msg << (detected ? 1 : 0) << ',' << (detected ? class_id : -1) << ','
            << std::fixed << std::setprecision(4) << (detected ? confidence : 0.0f) << ','
            << (detected ? cx : 0) << ',' << (detected ? cy : 0) << ',' << width << ',' << height;
        const std::string payload = msg.str();
        return sendto(fd_, payload.data(), payload.size(), 0,
                      reinterpret_cast<const sockaddr*>(&address_), sizeof(address_)) ==
               static_cast<ssize_t>(payload.size());
    }
private:
    int fd_ = -1;
    sockaddr_in address_{};
};
