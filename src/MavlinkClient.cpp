
#include "MavlinkClient.hpp"

#include <cerrno>
#include <cmath>
#include <iomanip>
#include <iostream>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

using Clock = std::chrono::steady_clock;

MavlinkClient::MavlinkClient(
    HealthMonitor& health,
    std::chrono::milliseconds heartbeatTimeout
)
    : health_(health),
      heartbeat_timeout_(heartbeatTimeout),
      last_heartbeat_(Clock::now()) {}


MavlinkClient::~MavlinkClient()
{
    if (socket_ >= 0)
        close(socket_);
}


bool MavlinkClient::openPort(int port)
{
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (socket_ < 0) {
        perror("socket");
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(
        socket_,
        reinterpret_cast<sockaddr*>(&addr),
        sizeof(addr)
    ) < 0) {
        perror("bind");
        close(socket_);
        socket_ = -1;
        return false;
    }

    std::cout
        << "[MAVLINK] Listening UDP "
        << port << "\n";

    return true;
}


void MavlinkClient::enableSitlTransmission(bool enabled)
{
    sitl_tx_enabled_ = enabled;

    std::cout
        << "[MAVLINK] Flight TX: "
        << (enabled ? "SITL ENABLED" : "DISABLED")
        << "\n";
}


bool MavlinkClient::flightTxEnabled() const
{
    return sitl_tx_enabled_;
}


bool MavlinkClient::sendMessage(
    const mavlink_message_t& message
)
{
    if (!sitl_tx_enabled_ ||
        socket_ < 0 ||
        !peer_valid_ ||
        !connected() ||
        system_id_ == 0) {
        return false;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    uint16_t length =
        mavlink_msg_to_send_buffer(buffer, &message);

    ssize_t sent = sendto(
        socket_,
        buffer,
        length,
        0,
        reinterpret_cast<const sockaddr*>(&peer_address_),
        sizeof(peer_address_)
    );

    if (sent != static_cast<ssize_t>(length)) {
        perror("[MAVLINK] sendto");
        return false;
    }

    return true;
}


bool MavlinkClient::requestGuided()
{
    if (!sitl_tx_enabled_ ||
        !connected() ||
        !armed() ||
        mode_ != 3) {
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
        << "[MAVLINK TX] Request AUTO -> GUIDED\n";

    return true;
}


bool MavlinkClient::sendBodyVelocity(
    float forwardMps,
    float rightMps,
    float downMps
)
{
    if (!sitl_tx_enabled_ ||
        !connected() ||
        !armed() ||
        mode_ != 4 ||
        !altitudeFresh()) {
        return false;
    }

    if (!std::isfinite(forwardMps) ||
        !std::isfinite(rightMps) ||
        !std::isfinite(downMps)) {
        return false;
    }

    constexpr float MAX_HORIZONTAL = 0.2f;
    constexpr float MAX_DOWN = 0.2f;

    const float horizontal =
        std::hypot(forwardMps, rightMps);

    if (horizontal > MAX_HORIZONTAL + 0.0001f ||
        std::abs(downMps) > MAX_DOWN + 0.0001f) {
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
        0.0f, 0.0f, 0.0f,
        forwardMps,
        rightMps,
        downMps,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f
    );

    return sendMessage(message);
}


void MavlinkClient::receive()
{
    if (socket_ < 0)
        return;

    uint8_t buffer[4096];
    mavlink_message_t message{};

    while (true) {
        sockaddr_in sender{};
        socklen_t senderLength = sizeof(sender);

        ssize_t count = recvfrom(
            socket_,
            buffer,
            sizeof(buffer),
            MSG_DONTWAIT,
            reinterpret_cast<sockaddr*>(&sender),
            &senderLength
        );

        if (count < 0) {
            if (errno == EINTR)
                continue;

            if (errno != EAGAIN &&
                errno != EWOULDBLOCK) {
                perror("recvfrom");
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

            if (message.msgid ==
                MAVLINK_MSG_ID_HEARTBEAT) {

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

                if (message.sysid != system_id_)
                    continue;

                if (!received_heartbeat_) {
                    std::cout
                        << "[MAVLINK] First heartbeat\n";
                }

                received_heartbeat_ = true;
                last_heartbeat_ = Clock::now();

                peer_address_ = sender;
                peer_valid_ = true;

                mode_ = hb.custom_mode;

                armed_ =
                    (hb.base_mode &
                     MAV_MODE_FLAG_SAFETY_ARMED) != 0;

                continue;
            }

            if (system_id_ == 0 ||
                message.sysid != system_id_ ||
                message.compid !=
                    MAV_COMP_ID_AUTOPILOT1) {
                continue;
            }

            if (message.msgid ==
                MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {

                mavlink_global_position_int_t pos{};

                mavlink_msg_global_position_int_decode(
                    &message,
                    &pos
                );

                relative_altitude_m_ =
                    static_cast<double>(pos.relative_alt)
                    / 1000.0;

                horizontal_speed_mps_ =
                    std::hypot(
                        static_cast<double>(pos.vx),
                        static_cast<double>(pos.vy)
                    ) / 100.0;

                // GLOBAL_POSITION_INT.vz is NED:
                // positive means downward.
                vertical_speed_mps_ =
                    static_cast<double>(pos.vz) / 100.0;

                const auto now = Clock::now();

                last_altitude_ = now;
                last_velocity_ = now;

                received_altitude_ = true;
                received_velocity_ = true;
            }

            if (message.msgid ==
                MAVLINK_MSG_ID_COMMAND_ACK) {

                mavlink_command_ack_t ack{};

                mavlink_msg_command_ack_decode(
                    &message,
                    &ack
                );

                if (ack.command == MAV_CMD_DO_SET_MODE) {
                    std::cout
                        << "[MAVLINK] Mode command ACK: "
                        << static_cast<int>(ack.result)
                        << "\n";
                }
            }

            health_.processMessage(message);
        }
    }
}


bool MavlinkClient::connected() const
{
    return received_heartbeat_ &&
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


bool MavlinkClient::altitudeFresh() const
{
    if (!connected() || !received_altitude_)
        return false;

    const auto elapsed =
        std::chrono::duration_cast<
            std::chrono::milliseconds
        >(Clock::now() - last_altitude_);

    return elapsed.count() < ALTITUDE_TIMEOUT_MS;
}


double MavlinkClient::relativeAltitudeM() const
{
    return relative_altitude_m_;
}


bool MavlinkClient::velocityFresh() const
{
    if (!connected() || !received_velocity_)
        return false;

    const auto elapsed =
        std::chrono::duration_cast<
            std::chrono::milliseconds
        >(Clock::now() - last_velocity_);

    return elapsed.count() < VELOCITY_TIMEOUT_MS;
}


double MavlinkClient::horizontalSpeedMps() const
{
    return horizontal_speed_mps_;
}


double MavlinkClient::verticalSpeedMps() const
{
    return vertical_speed_mps_;
}


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
            << (relative_altitude_m_ >= 3.0
                ? "ALTITUDE OK"
                : "TOO LOW")
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
        << (sitl_tx_enabled_
            ? "SITL ENABLED" : "DISABLED")
        << "\n"
        << "===========================\n";

    std::cout.unsetf(std::ios::floatfield);
}
