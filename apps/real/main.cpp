
#include "SerialMavlinkTransport.hpp"
#include "mavlink/ardupilotmega/mavlink.h"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {

using Clock =
    std::chrono::steady_clock;

volatile std::sig_atomic_t running = 1;

void stopHandler(int)
{
    running = 0;
}

bool parseBaudRate(
    const char* text,
    int& baudRate
)
{
    if (!text || *text == '\0')
        return false;

    char* end = nullptr;
    errno = 0;

    const long value =
        std::strtol(text, &end, 10);

    if (
        errno != 0 ||
        *end != '\0' ||
        value < 1 ||
        value > 1000000
    ) {
        return false;
    }

    baudRate = static_cast<int>(value);
    return true;
}

} // namespace


int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cerr
            << "Usage: "
            << argv[0]
            << " <serial-device> <baud-rate>\n"
            << "Example: "
            << argv[0]
            << " /dev/ttyTHS1 57600\n";

        return 1;
    }

    int baudRate = 0;

    if (!parseBaudRate(argv[2], baudRate)) {
        std::cerr
            << "[ERROR] Invalid baud rate\n";
        return 1;
    }

    std::signal(SIGINT, stopHandler);
    std::signal(SIGTERM, stopHandler);

    std::cout
        << "[CONFIG] REAL TELEMETRY ONLY\n"
        << "[CONFIG] Flight TX: NOT IMPLEMENTED\n";

    SerialMavlinkTransport serial;

    if (!serial.openPort(
        argv[1],
        baudRate
    )) {
        return 1;
    }

    mavlink_message_t message{};
    mavlink_status_t parserStatus{};

    uint8_t buffer[4096];

    bool heartbeatReceived = false;
    bool positionReceived = false;

    uint8_t systemId = 0;

    bool armed = false;
    uint32_t mode = 0;

    double altitudeM = 0.0;
    double horizontalMps = 0.0;
    double verticalMps = 0.0;

    auto lastHeartbeat = Clock::now();
    auto lastPosition = Clock::now();
    auto lastReport = Clock::now();

    std::cout
        << "[REAL] Waiting for "
        << "Pixhawk MAVLink telemetry...\n";

    while (running) {

        const ssize_t count = serial.receive(
            buffer,
            sizeof(buffer)
        );

        if (count < 0) {
            if (
                errno != EAGAIN &&
                errno != EWOULDBLOCK &&
                errno != EINTR
            ) {
                std::cerr
                    << "[SERIAL] Read error: "
                    << std::strerror(errno)
                    << "\n";
                break;
            }
        }

        if (count > 0) {
            for (
                ssize_t i = 0;
                i < count;
                ++i
            ) {
                if (!mavlink_parse_char(
                    MAVLINK_COMM_1,
                    buffer[i],
                    &message,
                    &parserStatus
                )) {
                    continue;
                }

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
                        message.compid !=
                            MAV_COMP_ID_AUTOPILOT1
                    ) {
                        continue;
                    }

                    if (systemId == 0)
                        systemId = message.sysid;

                    if (message.sysid != systemId)
                        continue;

                    if (!heartbeatReceived) {
                        std::cout
                            << "[MAVLINK] First "
                            << "autopilot HEARTBEAT\n";
                    }

                    heartbeatReceived = true;
                    lastHeartbeat = Clock::now();

                    armed =
                        (hb.base_mode &
                         MAV_MODE_FLAG_SAFETY_ARMED)
                        != 0;

                    mode = hb.custom_mode;
                }

                if (
                    systemId == 0 ||
                    message.sysid != systemId ||
                    message.compid !=
                        MAV_COMP_ID_AUTOPILOT1
                ) {
                    continue;
                }

                if (
                    message.msgid ==
                    MAVLINK_MSG_ID_GLOBAL_POSITION_INT
                ) {
                    mavlink_global_position_int_t pos{};

                    mavlink_msg_global_position_int_decode(
                        &message,
                        &pos
                    );

                    altitudeM =
                        static_cast<double>(
                            pos.relative_alt
                        ) / 1000.0;

                    horizontalMps = std::hypot(
                        static_cast<double>(pos.vx),
                        static_cast<double>(pos.vy)
                    ) / 100.0;

                    verticalMps =
                        static_cast<double>(
                            pos.vz
                        ) / 100.0;

                    positionReceived = true;
                    lastPosition = Clock::now();
                }
            }
        }

        const auto now = Clock::now();

        if (
            now - lastReport >=
            std::chrono::seconds(1)
        ) {
            const bool heartbeatFresh =
                heartbeatReceived &&
                now - lastHeartbeat <
                    std::chrono::seconds(5);

            const bool positionFresh =
                positionReceived &&
                now - lastPosition <
                    std::chrono::seconds(5);

            std::cout
                << "\n[REAL TELEMETRY]\n"
                << "  Link      : "
                << (heartbeatFresh
                    ? "CONNECTED"
                    : "NO HEARTBEAT")
                << "\n";

            if (heartbeatFresh) {
                std::cout
                    << "  Armed     : "
                    << (armed ? "YES" : "NO")
                    << "\n"
                    << "  Mode      : "
                    << mode
                    << " (ArduCopter custom_mode)\n";
            } else {
                std::cout
                    << "  Armed/Mode: STALE\n";
            }

            if (
                heartbeatFresh &&
                positionFresh
            ) {
                std::cout
                    << std::fixed
                    << std::setprecision(2)
                    << "  Rel Alt   : "
                    << altitudeM
                    << " m\n"
                    << "  Horiz Spd : "
                    << horizontalMps
                    << " m/s\n"
                    << "  Vert Spd  : "
                    << verticalMps
                    << " m/s (down +)\n";
            } else {
                std::cout
                    << "  Position  : "
                    << "NO DATA / STALE\n";
            }

            std::cout
                << "  Flight TX : DISABLED "
                << "(receive-only)\n";

            lastReport = now;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(10)
        );
    }

    std::cout
        << "\n[REAL] Telemetry receiver stopped\n";

    return 0;
}
