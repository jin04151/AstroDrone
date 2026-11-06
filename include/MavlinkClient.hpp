
#pragma once

#include <chrono>
#include <cstdint>

#include "HealthMonitor.hpp"
#include "UdpMavlinkTransport.hpp"
#include "mavlink/ardupilotmega/mavlink.h"

class MavlinkClient {
public:
    explicit MavlinkClient(
        HealthMonitor& health,
        std::chrono::milliseconds heartbeatTimeout
    );

    ~MavlinkClient();

    MavlinkClient(
        const MavlinkClient&
    ) = delete;

    MavlinkClient& operator=(
        const MavlinkClient&
    ) = delete;

    bool openPort(int port);
    void receive();

    bool connected() const;
    bool armed() const;
    uint32_t mode() const;

    bool altitudeFresh() const;
    double relativeAltitudeM() const;

    bool velocityFresh() const;
    double horizontalSpeedMps() const;
    double verticalSpeedMps() const;

    void enableSitlTransmission(
        bool enabled
    );

    bool flightTxEnabled() const;

    bool requestGuided();

    bool sendBodyVelocity(
        float forwardMps,
        float rightMps,
        float downMps
    );

    void printAltitudeStatus() const;

private:
    using Clock =
        std::chrono::steady_clock;

    bool sendMessage(
        const mavlink_message_t& message
    );

    HealthMonitor& health_;

    std::chrono::milliseconds
        heartbeat_timeout_;

    // UDP transport is separated
    // from MAVLink message processing.
    UdpMavlinkTransport transport_;

    bool sitl_tx_enabled_ = false;

    bool received_heartbeat_ = false;
    bool armed_ = false;

    uint32_t mode_ = 0;
    uint8_t system_id_ = 0;

    Clock::time_point last_heartbeat_{};

    bool received_altitude_ = false;
    double relative_altitude_m_ = 0.0;
    Clock::time_point last_altitude_{};

    bool received_velocity_ = false;
    double horizontal_speed_mps_ = 0.0;
    double vertical_speed_mps_ = 0.0;
    Clock::time_point last_velocity_{};

    static constexpr int
        ALTITUDE_TIMEOUT_MS = 5000;

    static constexpr int
        VELOCITY_TIMEOUT_MS = 3000;

    mavlink_status_t parser_status_{};
};
