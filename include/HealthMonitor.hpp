
#pragma once

#include <chrono>
#include <cstdint>

#include "mavlink/ardupilotmega/mavlink.h"

enum class CheckStatus {
    PASS,
    FAIL,
    NO_DATA
};

struct HealthReport {
    CheckStatus battery = CheckStatus::NO_DATA;
    CheckStatus gps = CheckStatus::NO_DATA;
    CheckStatus rc = CheckStatus::NO_DATA;

    bool ready = false;
};

class HealthMonitor {
public:
    explicit HealthMonitor(
        std::chrono::milliseconds telemetryTimeout
    );

    void processMessage(
        const mavlink_message_t& msg
    );

    HealthReport check() const;

    void printStatus() const;

private:
    using Clock = std::chrono::steady_clock;

    struct BatteryData {
        bool received = false;
        double voltage = -1.0;
        int remaining = -1;
        Clock::time_point updated{};
    };

    struct GpsData {
        bool received = false;
        int fix = 0;
        int satellites = 255;
        Clock::time_point updated{};
    };

    struct RcData {
        bool received = false;
        int channels = 0;
        int rssi = 255;
        Clock::time_point updated{};
    };

    BatteryData battery_;
    GpsData gps_;
    RcData rc_;

    std::chrono::milliseconds telemetry_timeout_;

    static constexpr int MIN_BATTERY_PERCENT = 20;
    static constexpr int MIN_GPS_FIX = 3;
    static constexpr int MIN_SATELLITES = 6;

    bool isFresh(
        bool received,
        Clock::time_point updated
    ) const;

    static const char* statusName(
        CheckStatus status
    );
};
