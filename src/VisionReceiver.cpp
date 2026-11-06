
#include "VisionReceiver.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>

VisionReceiver::~VisionReceiver()
{
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }
}

bool VisionReceiver::openPort(uint16_t port)
{
    if (socket_fd_ >= 0) {
        std::cerr << "[VISION] Port already open\n";
        return false;
    }

    socket_fd_ = socket(
        AF_INET,
        SOCK_DGRAM,
        0
    );

    if (socket_fd_ < 0) {
        perror("[VISION] socket");
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr =
        htonl(INADDR_LOOPBACK);

    if (bind(
        socket_fd_,
        reinterpret_cast<sockaddr*>(&address),
        sizeof(address)
    ) < 0) {

        perror("[VISION] bind");
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    int flags = fcntl(
        socket_fd_,
        F_GETFL,
        0
    );

    if (flags < 0 ||
        fcntl(
            socket_fd_,
            F_SETFL,
            flags | O_NONBLOCK
        ) < 0) {

        perror("[VISION] fcntl");

        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    std::cout
        << "[VISION] Listening UDP 127.0.0.1:"
        << port << "\n";

    return true;
}

// CSV protocol:
// detected,class_id,confidence,cx,cy,width,height
bool VisionReceiver::parseDetection(
    const std::string& message,
    Detection& result
) const
{
    std::stringstream ss(message);

    std::string fields[7];
    std::string extra;

    for (int i = 0; i < 7; ++i) {
        if (!std::getline(ss, fields[i], ',')) {
            return false;
        }
    }

    if (std::getline(ss, extra, ',')) {
        return false;
    }

    try {
        auto parseInt = [](const std::string& s) {
            size_t pos = 0;
            int value = std::stoi(s, &pos);

            if (pos != s.size()) {
                throw std::invalid_argument("integer");
            }

            return value;
        };

        auto parseFloat = [](const std::string& s) {
            size_t pos = 0;
            float value = std::stof(s, &pos);

            if (pos != s.size() ||
                !std::isfinite(value)) {

                throw std::invalid_argument("float");
            }

            return value;
        };

        int detected = parseInt(fields[0]);

        if (detected != 0 && detected != 1) {
            return false;
        }

        result.detected = (detected == 1);
        result.class_id = parseInt(fields[1]);
        result.confidence = parseFloat(fields[2]);

        result.cx = parseInt(fields[3]);
        result.cy = parseInt(fields[4]);

        result.width = parseInt(fields[5]);
        result.height = parseInt(fields[6]);

        if (result.width <= 0 ||
            result.height <= 0 ||
            result.width > 16384 ||
            result.height > 16384) {

            return false;
        }

        if (result.confidence < 0.0f ||
            result.confidence > 1.0f) {

            return false;
        }

        if (result.detected) {

            if (result.class_id < 0 ||
                result.cx < 0 ||
                result.cy < 0 ||
                result.cx >= result.width ||
                result.cy >= result.height) {

                return false;
            }

        } else {

            if (result.class_id != -1 ||
                result.confidence != 0.0f) {

                return false;
            }
        }

    } catch (...) {
        return false;
    }

    return true;
}

// Validate consecutive YOLO detections.
void VisionReceiver::updateValidation(
    const Detection& detection
)
{
    bool valid =
        detection.detected &&
        detection.class_id == TARGET_CLASS_ID &&
        detection.confidence >= MIN_CONFIDENCE;

    if (!valid) {
        consecutive_detections_ = 0;
        return;
    }

    // Cap at the required number.
    if (consecutive_detections_ < REQUIRED_FRAMES) {
        ++consecutive_detections_;
    }
}

// Non-blocking UDP receiver.
void VisionReceiver::receive()
{
    if (socket_fd_ < 0) {
        return;
    }

    // Reset the validation streak if the
    // previous data has already expired.
    if (!isFresh()) {
        consecutive_detections_ = 0;
    }

    // Limit packets processed per main loop.
    for (int i = 0; i < 32; ++i) {

        char buffer[512]{};

        ssize_t bytes = recvfrom(
            socket_fd_,
            buffer,
            sizeof(buffer),
            MSG_DONTWAIT | MSG_TRUNC,
            nullptr,
            nullptr
        );

        if (bytes < 0) {

            if (errno == EAGAIN ||
                errno == EWOULDBLOCK) {
                break;
            }

            if (errno == EINTR) {
                continue;
            }

            perror("[VISION] recvfrom");
            break;
        }

        if (bytes == 0 ||
            bytes >= static_cast<ssize_t>(
                sizeof(buffer))) {

            ++invalid_count_;
            consecutive_detections_ = 0;
            continue;
        }

        std::string message(
            buffer,
            static_cast<size_t>(bytes)
        );

        Detection detection{};

        if (!parseDetection(
            message,
            detection
        )) {

            ++invalid_count_;
            consecutive_detections_ = 0;
            continue;
        }

        // Break the streak if there was
        // a gap longer than the timeout.
        if (!isFresh()) {
            consecutive_detections_ = 0;
        }

        latest_ = detection;

        received_ = true;
        last_received_ = Clock::now();

        ++packet_count_;

        updateValidation(detection);
    }
}

bool VisionReceiver::isFresh() const
{
    if (!received_) {
        return false;
    }

    auto elapsed =
        std::chrono::duration_cast<
            std::chrono::milliseconds
        >(
            Clock::now() - last_received_
        );

    return elapsed.count() < TIMEOUT_MS;
}

bool VisionReceiver::targetDetected() const
{
    return isFresh() && latest_.detected;
}

bool VisionReceiver::targetVerified() const
{
    return isFresh() &&
           latest_.detected &&
           latest_.class_id == TARGET_CLASS_ID &&
           latest_.confidence >= MIN_CONFIDENCE &&
           consecutive_detections_ >= REQUIRED_FRAMES;
}

void VisionReceiver::printStatus() const
{
    std::cout
        << "\n===== VISION STATUS =====\n";

    if (!received_) {
        std::cout
            << "Connection : NO DATA\n";

    } else if (!isFresh()) {
        std::cout
            << "Connection : STALE\n";

    } else {
        std::cout
            << "Connection : OK\n";
    }

    std::cout
        << "Packets    : " << packet_count_ << "\n";

    std::cout
        << "Invalid    : " << invalid_count_ << "\n";

    if (!isFresh()) {

        std::cout
            << "Target     : UNAVAILABLE\n";

        std::cout
            << "Consecutive: 0 / "
            << REQUIRED_FRAMES << "\n";

        std::cout
            << "Validation : NO DATA\n";

    } else if (!latest_.detected) {

        std::cout
            << "Target     : NOT DETECTED\n";

        std::cout
            << "Consecutive: 0 / "
            << REQUIRED_FRAMES << "\n";

        std::cout
            << "Validation : NOT DETECTED\n";

    } else {

        float error_x =
            (latest_.cx -
             latest_.width / 2.0f) /
            (latest_.width / 2.0f);

        float error_y =
            (latest_.cy -
             latest_.height / 2.0f) /
            (latest_.height / 2.0f);

        std::cout
            << "Target     : DETECTED\n";

        std::cout
            << "Class ID   : "
            << latest_.class_id << "\n";

        std::cout
            << "Confidence : "
            << std::fixed
            << std::setprecision(3)
            << latest_.confidence << "\n";

        std::cout
            << "Center     : ("
            << latest_.cx << ", "
            << latest_.cy << ")\n";

        std::cout
            << "Image      : "
            << latest_.width << " x "
            << latest_.height << "\n";

        std::cout
            << "Error X/Y  : ("
            << error_x << ", "
            << error_y << ")\n";

        std::cout
            << "Consecutive: "
            << consecutive_detections_
            << " / " << REQUIRED_FRAMES
            << "\n";

        if (targetVerified()) {

            std::cout
                << "Validation : VERIFIED\n";

        } else if (
            latest_.class_id != TARGET_CLASS_ID
        ) {

            std::cout
                << "Validation : WRONG CLASS\n";

        } else if (
            latest_.confidence < MIN_CONFIDENCE
        ) {

            std::cout
                << "Validation : LOW CONFIDENCE\n";

        } else {

            std::cout
                << "Validation : PENDING\n";
        }
    }

    std::cout
        << "Min Conf   : "
        << MIN_CONFIDENCE << "\n";

    std::cout
        << "Flight TX  : DISABLED\n";

    std::cout
        << "=========================\n";
}
