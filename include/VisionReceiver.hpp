
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

struct Detection {
    bool detected = false;
    int class_id = -1;
    float confidence = 0.0f;

    int cx = 0;
    int cy = 0;
    int width = 0;
    int height = 0;
};

class VisionReceiver {
public:
    VisionReceiver() = default;
    ~VisionReceiver();

    VisionReceiver(const VisionReceiver&) = delete;
    VisionReceiver& operator=(const VisionReceiver&) = delete;

    bool openPort(uint16_t port = 14560);
    void receive();

    bool isFresh() const;
    bool targetDetected() const;
    bool targetVerified() const;

    // Most recently received YOLO coordinates.
    // Always check isFresh() before using them.
    Detection latestDetection() const {
        return latest_;
    }

    void printStatus() const;

private:
    using Clock = std::chrono::steady_clock;

    bool parseDetection(
        const std::string& message,
        Detection& result
    ) const;

    void updateValidation(const Detection& detection);

    int socket_fd_ = -1;

    Detection latest_{};

    bool received_ = false;
    Clock::time_point last_received_{};

    unsigned long packet_count_ = 0;
    unsigned long invalid_count_ = 0;

    int consecutive_detections_ = 0;

    static constexpr int TARGET_CLASS_ID = 0;
    static constexpr float MIN_CONFIDENCE = 0.60f;
    static constexpr int REQUIRED_FRAMES = 5;
    static constexpr int TIMEOUT_MS = 1000;
};
