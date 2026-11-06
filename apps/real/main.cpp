
#include "MavlinkClient.hpp"
#include "SerialMavlinkTransport.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;

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
    if (text == nullptr || *text == '\0')
        return false;

    char* end = nullptr;
    errno = 0;

    const long value = std::strtol(
        text,
        &end,
        10
    );

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
            << " <device> <baud-rate>\n"
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
        << "[CONFIG] Mode: REAL\n"
        << "[CONFIG] Telemetry: RECEIVE ONLY\n"
        << "[CONFIG] Flight TX: DISABLED\n";

    // Physical UART device.
    // SerialMavlinkTransport opens it O_RDONLY.
    SerialMavlinkTransport serial;

    if (!serial.openPort(
        argv[1],
        baudRate
    )) {
        return 1;
    }

    // Shared MAVLink parser.
    MavlinkClient mavlink(
        serial,
        std::chrono::seconds(10)
    );

    // Explicitly retain receive-only mode.
    mavlink.enableSitlTransmission(false);

    auto lastReport = Clock::now();

    std::cout
        << "[REAL] Waiting for "
        << "Pixhawk MAVLink telemetry...\n";

    while (running) {
        mavlink.receive();

        const auto now = Clock::now();

        if (
            now - lastReport >=
            std::chrono::seconds(1)
        ) {
            std::cout
                << "\n===== REAL TELEMETRY =====\n";

            const bool connected =
                mavlink.connected();

            std::cout
                << "Connection : "
                << (connected ? "OK" : "NO HEARTBEAT")
                << "\n";

            if (connected) {
                std::cout
                    << "Armed      : "
                    << (mavlink.armed() ? "YES" : "NO")
                    << "\n"
                    << "Mode       : "
                    << mavlink.mode()
                    << "\n";
            } else {
                std::cout
                    << "Armed/Mode : NO DATA / STALE\n";
            }

            // Reuses the existing altitude and
            // velocity freshness checks.
            mavlink.printAltitudeStatus();

            std::cout
                << "Flight TX  : DISABLED\n"
                << "==========================\n";

            lastReport = now;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(10)
        );
    }

    std::cout
        << "[REAL] Receiver stopped\n";

    return 0;
}
