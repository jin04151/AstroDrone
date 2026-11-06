
#include "MavlinkClient.hpp"

#include <cerrno>
#include <cmath>
#include <iomanip>
#include <iostream>

using Clock = std::chrono::steady_clock;


MavlinkClient::MavlinkClient(
    HealthMonitor& health,
    std::chrono::milliseconds heartbeatTimeout
)
    : health_(health),
      heartbeat_timeout_(heartbeatTimeout),
      last_heartbeat_(Clock::now())
{
}


MavlinkClient::~MavlinkClient() = default;


// ========================================
// UDP connection
// ========================================

bool MavlinkClient::openPort(int port)
{
    return transport_.openPort(port);
}


// ========================================
// Flight transmission configuration
// ========================================

void MavlinkClient::enableSitlTransmission(
    bool enabled
)
{
    sitl_tx_enabled_ = enabled;

    std::cout
        << "[MAVLINK] Flight TX: "
        << (
            enabled
                ? "SITL ENABLED"
                : "DISABLED"
        )
        << "\n";
}


bool MavlinkClient::flightTxEnabled() const
{
    return sitl_tx_enabled_;
}


// ========================================
// MAVLink transmission
// ========================================

bool MavlinkClient::sendMessage(
    const mavlink_message_t& message
)
{
    if (
        !sitl_tx_enabled_ ||
        !transport_.isOpen() ||
        !transport_.hasPeer() ||
        !connected() ||
        system_id_ == 0
    ) {
        return false;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    const uint16_t length =
        mavlink_msg_to_send_buffer(
            buffer,
            &message
        );

    return transport_.send(
        buffer,
        length
    );
}


// ========================================
// AUTO -> GUIDED
// ========================================

bool MavlinkClient::requestGuided()
{
    if (
        !sitl_tx_enabled_ ||
        !connected() ||
        !armed() ||
        mode_ != 3
    ) {
        return false;
    }

    mavlink_message_t message{};

    mavlink_msg_command_long_pack(
        255,
        190,
        &message,
        system_id_,
        MAV_COMP_ID_AUTOPILOT1,
        MAV_CMD_DO_SET_MODE,
        0,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );

    if (!sendMessage(message))
        return false;

    std::cout
        << "[MAVLINK TX] "
        << "Request AUTO -> GUIDED\n";

    return true;
}


// ========================================
// BODY velocity transmission
// ========================================

bool MavlinkClient::sendBodyVelocity(
    float forwardMps,
    float rightMps,
    float downMps
)
{
    if (
        !sitl_tx_enabled_ ||
        !connected() ||
        !armed() ||
        mode_ != 4 ||
        !altitudeFresh()
    ) {
        return false;
    }

    if (
        !std::isfinite(forwardMps) ||
        !std::isfinite(rightMps) ||
        !std::isfinite(downMps)
    ) {
        return false;
    }

    constexpr float MAX_HORIZONTAL = 0.2f;
    constexpr float MAX_DOWN = 0.2f;

    const float horizontal =
        std::hypot(
            forwardMps,
            rightMps
        );

    if (
        horizontal >
            MAX_HORIZONTAL + 0.0001f ||
        std::abs(downMps) >
            MAX_DOWN + 0.0001f
    ) {
        return false;
    }

    mavlink_message_t message{};

    mavlink_msg_set_position_target_local_ned_pack(
        255,
        190,
        &message,
        0,
        system_id_,
        MAV_COMP_ID_AUTOPILOT1,
        MAV_FRAME_BODY_OFFSET_NED,
        3527,
        0.0f,
        0.0f,
        0.0f,
        forwardMps,
        rightMps,
        downMps,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );

    return sendMessage(message);
}


// ========================================
// MAVLink receive + parsing
// ========================================

void MavlinkClient::receive()
{
    if (!transport_.isOpen())
        return;

    uint8_t buffer[4096];
    mavlink_message_t message{};

    while (true) {

        const ssize_t count =
            transport_.receive(
                buffer,
                sizeof(buffer)
            );

        if (count < 0) {

            if (errno == EINTR)
                continue;

            if (
                errno != EAGAIN &&
                errno != EWOULDBLOCK
            ) {
                perror("[MAVLINK] receive");
            }

            break;
        }

        if (count == 0)
            break;

        for (ssize_t i = 0; i < count; ++i) {

            if (!mavlink_parse_char(
                MAVLINK_COMM_0,
                buffer[i],
                &message,
                &parser_status_
            )) {
                continue;
            }

            // ----------------------------
            // HEARTBEAT
            // ----------------------------

            if (
                message.msgid ==
                MAVLINK_MSG_ID_HEARTBEAT
            ) {
                mavlink_heartbeat_t hb{};

                mavlink_msg_heartbeat_decode(
                    &message,
                    &hb
                );

                if (
                    hb.autopilot !=
                        MAV_AUTOPILOT_ARDUPILOTMEGA ||
                    hb.type != MAV_TYPE_QUADROTOR ||
                    message.compid !=
                        MAV_COMP_ID_AUTOPILOT1
                ) {
                    continue;
                }

                if (system_id_ == 0)
                    system_id_ = message.sysid;

                if (
                    message.sysid !=
                    system_id_
                ) {
                    continue;
                }

                if (!received_heartbeat_) {
                    std::cout
                        << "[MAVLINK] "
                        << "First heartbeat\n";
                }

                received_heartbeat_ = true;
                last_heartbeat_ = Clock::now();

                // Store the address only
                // after validating HEARTBEAT.
                transport_.confirmLastSender();

                mode_ = hb.custom_mode;

                armed_ =
                    (
                        hb.base_mode &
                        MAV_MODE_FLAG_SAFETY_ARMED
                    ) != 0;

                continue;
            }

            // ----------------------------
            // Validate autopilot sender
            // ----------------------------

            if (
                system_id_ == 0 ||
                message.sysid != system_id_ ||
                message.compid !=
                    MAV_COMP_ID_AUTOPILOT1
            ) {
                continue;
            }

            // ----------------------------
            // Altitude and velocity
            // ----------------------------

            if (
                message.msgid ==
                MAVLINK_MSG_ID_GLOBAL_POSITION_INT
            ) {
                mavlink_global_position_int_t pos{};

                mavlink_msg_global_position_int_decode(
                    &message,
                    &pos
                );

                relative_altitude_m_ =
                    static_cast<double>(
                        pos.relative_alt
                    ) / 1000.0;

                horizontal_speed_mps_ =
                    std::hypot(
                        static_cast<double>(pos.vx),
                        static_cast<double>(pos.vy)
                    ) / 100.0;

                // NED positive Z = down.
                vertical_speed_mps_ =
                    static_cast<double>(
                        pos.vz
                    ) / 100.0;

                const auto now = Clock::now();

                last_altitude_ = now;
                last_velocity_ = now;

                received_altitude_ = true;
                received_velocity_ = true;
            }

            // ----------------------------
            // Mode command acknowledgement
            // ----------------------------

            if (
                message.msgid ==
                MAVLINK_MSG_ID_COMMAND_ACK
            ) {
                mavlink_command_ack_t ack{};

                mavlink_msg_command_ack_decode(
                    &message,
                    &ack
                );

                if (
                    ack.command ==
                    MAV_CMD_DO_SET_MODE
                ) {
                    std::cout
                        << "[MAVLINK] "
                        << "Mode command ACK: "
                        << static_cast<int>(
                            ack.result
                        )
                        << "\n";
                }
            }

            health_.processMessage(message);
        }
    }
}


// ========================================
// Flight controller state
// ========================================

bool MavlinkClient::connected() const
{
    return
        received_heartbeat_ &&
        Clock::now() - last_heartbeat_ <
            heartbeat_timeout_;
}


bool MavlinkClient::armed() const
{
    return armed_;
}


uint32_t MavlinkClient::mode() const
{
    return mode_;
}


// ========================================
// Altitude
// ========================================

bool MavlinkClient::altitudeFresh() const
{
    if (
        !connected() ||
        !received_altitude_
    ) {
        return false;
    }

    const auto elapsed =
        std::chrono::duration_cast<
            std::chrono::milliseconds
        >(
            Clock::now() - last_altitude_
        );

    return elapsed.count() <
        ALTITUDE_TIMEOUT_MS;
}


double MavlinkClient::relativeAltitudeM() const
{
    return relative_altitude_m_;
}


// ========================================
// Velocity
// ========================================

bool MavlinkClient::velocityFresh() const
{
    if (
        !connected() ||
        !received_velocity_
    ) {
        return false;
    }

    const auto elapsed =
        std::chrono::duration_cast<
            std::chrono::milliseconds
        >(
            Clock::now() - last_velocity_
        );

    return elapsed.count() <
        VELOCITY_TIMEOUT_MS;
}


double MavlinkClient::horizontalSpeedMps() const
{
    return horizontal_speed_mps_;
}


double MavlinkClient::verticalSpeedMps() const
{
    return vertical_speed_mps_;
}


// ========================================
// Status
// ========================================

void MavlinkClient::printAltitudeStatus() const
{
    std::cout
        << "\n===== ALTITUDE STATUS =====\n";

    if (!altitudeFresh()) {

        std::cout
            << "Relative Alt : NO DATA / STALE\n"
            << "Search >= 3m : NOT ALLOWED\n";

    } else {

        std::cout
            << std::fixed
            << std::setprecision(2)
            << "Relative Alt : "
            << relative_altitude_m_
            << " m\n"
            << "Search >= 3m : "
            << (
                relative_altitude_m_ >= 3.0
                    ? "ALTITUDE OK"
                    : "TOO LOW"
            )
            << "\n";
    }

    if (!velocityFresh()) {

        std::cout
            << "Velocity     : NO DATA / STALE\n";

    } else {

        std::cout
            << "Horiz speed  : "
            << horizontal_speed_mps_
            << " m/s\n"
            << "Vert speed   : "
            << vertical_speed_mps_
            << " m/s (down +)\n";
    }

    std::cout
        << "Flight TX    : "
        << (
            sitl_tx_enabled_
                ? "SITL ENABLED"
                : "DISABLED"
        )
        << "\n"
        << "===========================\n";

    std::cout.unsetf(
        std::ios::floatfield
    );
}
