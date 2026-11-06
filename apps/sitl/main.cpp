
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <chrono>
#include <poll.h>

#include "HealthMonitor.hpp"
#include "MavlinkClient.hpp"
#include "MissionManager.hpp"
#include "SafetyManager.hpp"
#include "VisionReceiver.hpp"
#include "FlightPhaseManager.hpp"

using Clock = std::chrono::steady_clock;

struct RuntimeConfig {
    std::string mode;
    std::chrono::milliseconds heartbeatTimeout;
    std::chrono::milliseconds telemetryTimeout;
    bool enableFlightTx = false;
};

bool parseConfig(
    int argc,
    char* argv[],
    RuntimeConfig& config
)
{
    if (argc < 2 || argc > 3)
        return false;

    const std::string arg = argv[1];

    if (arg == "--sitl") {
        config.mode = "SITL";
        config.heartbeatTimeout =
            std::chrono::seconds(10);
        config.telemetryTimeout =
            std::chrono::seconds(10);

        if (argc == 3) {
            if (std::string(argv[2]) !=
                "--enable-flight-tx") {
                return false;
            }

            config.enableFlightTx = true;
        }

        return true;
    }

    if (arg == "--real" && argc == 2) {
        config.mode = "REAL";
        config.heartbeatTimeout =
            std::chrono::seconds(3);
        config.telemetryTimeout =
            std::chrono::seconds(5);
        config.enableFlightTx = false;
        return true;
    }

    return false;
}


PhaseInput makePhaseInput(
    const MavlinkClient& mavlink,
    const VisionReceiver& vision,
    const SafetyManager& safety,
    const MissionManager& mission
)
{
    PhaseInput in{};

    in.connected = mavlink.connected();
    in.manualLockout = safety.isManualLocked();

    in.missionActive =
        mission.state() ==
        MissionState::MISSION_ACTIVE;

    in.armed = mavlink.armed();

    in.inAuto =
        in.connected &&
        mavlink.mode() == 3;

    in.inGuided =
        in.connected &&
        mavlink.mode() == 4;

    in.altitudeValid =
        mavlink.altitudeFresh();

    if (in.altitudeValid) {
        in.relativeAltitudeM =
            mavlink.relativeAltitudeM();
    }

    in.velocityValid =
        mavlink.velocityFresh();

    if (in.velocityValid) {
        in.horizontalSpeedMps =
            mavlink.horizontalSpeedMps();

        in.verticalSpeedMps =
            mavlink.verticalSpeedMps();
    }

    in.visionFresh = vision.isFresh();
    in.targetVisible = vision.targetDetected();
    in.targetVerified = vision.targetVerified();

    if (in.visionFresh) {
        const Detection d =
            vision.latestDetection();

        in.targetCx = d.cx;
        in.targetCy = d.cy;
        in.imageWidth = d.width;
        in.imageHeight = d.height;
    }

    return in;
}


int main(int argc, char* argv[])
{
    RuntimeConfig config{};

    if (!parseConfig(argc, argv, config)) {
        std::cerr
            << "Usage:\n"
            << "  ./build/control --sitl\n"
            << "  ./build/control --sitl --enable-flight-tx\n"
            << "  ./build/control --real\n";
        return 1;
    }

    std::cout
        << "[CONFIG] Mode: "
        << config.mode << "\n"
        << "[CONFIG] Heartbeat timeout: "
        << config.heartbeatTimeout.count()
        << " ms\n"
        << "[CONFIG] Telemetry timeout: "
        << config.telemetryTimeout.count()
        << " ms\n"
        << "[CONFIG] Flight TX: "
        << (config.enableFlightTx
            ? "SITL ENABLED"
            : "DISABLED")
        << "\n";

    HealthMonitor health(
        config.telemetryTimeout
    );

    MavlinkClient mavlink(
        health,
        config.heartbeatTimeout
    );

    SafetyManager safety;
    MissionManager mission(safety);
    VisionReceiver vision;
    FlightPhaseManager flightPhase;

    if (!mavlink.openPort(14552) ||
        !vision.openPort(14560)) {
        return 1;
    }

    mavlink.enableSitlTransmission(
        config.enableFlightTx
    );

    std::cout
        << "\n[INIT] Control C++\n"
        << "[COMMANDS] "
        << "APPROACH VERTICAL / APPROACH DIAGONAL / "
        << "FLY / MANUAL / RESET / STATUS / EXIT\n";

    if (config.enableFlightTx) {
        std::cout
            << "[WARNING] SITL ACTIVE FLIGHT CONTROL\n"
            << "[WARNING] Manual takeover has priority\n";
    }

    uint32_t previous_mode = 0;
    bool mode_initialized = false;

    bool guidedRequestSent = false;
    bool guidedTransferFailed = false;
    bool guidedTransferConfirmed = false;

    int guidedAttempts = 0;

    Clock::time_point firstGuidedRequest{};
    Clock::time_point lastGuidedRequest{};

    auto lastVelocityTx = Clock::now();
    auto lastPlanLog = Clock::now();

    constexpr int MAX_GUIDED_ATTEMPTS = 5;
    constexpr int GUIDED_RETRY_MS = 1500;
    constexpr int GUIDED_TIMEOUT_MS = 9000;
    constexpr int VELOCITY_INTERVAL_MS = 200;

    while (true) {
        mavlink.receive();
        vision.receive();

        bool connected = mavlink.connected();

        HealthReport health_report =
            health.check();

        // 1. External mode change detection.
        if (connected) {
            const uint32_t current_mode =
                mavlink.mode();

            if (!mode_initialized) {
                previous_mode = current_mode;
                mode_initialized = true;

            } else if (
                previous_mode != current_mode
            ) {
                mission.onModeChanged(
                    previous_mode,
                    current_mode
                );

                previous_mode = current_mode;
            }
        } else {
            mode_initialized = false;
        }

        // 2. Existing mission / safety state.
        mission.update(
            connected,
            health_report
        );

        PhaseInput phaseInput =
            makePhaseInput(
                mavlink,
                vision,
                safety,
                mission
            );

        flightPhase.update(phaseInput);

        if (
            phaseInput.missionActive &&
            phaseInput.connected &&
            !phaseInput.manualLockout &&
            phaseInput.armed &&
            phaseInput.inAuto &&
            flightPhase.phase() ==
                FlightPhase::IDLE
        ) {
            flightPhase.startAutoSearch(
                phaseInput
            );
        }

        const auto now = Clock::now();

        // 3. Command authorization.
        const bool flightAuthorized =
            config.enableFlightTx &&
            phaseInput.connected &&
            !phaseInput.manualLockout &&
            phaseInput.missionActive &&
            phaseInput.armed &&
            phaseInput.altitudeValid;

        if (!flightAuthorized) {
            guidedTransferConfirmed = false;
        }

        // 4. Automatic AUTO -> GUIDED request.
        if (
            flightAuthorized &&
            phaseInput.inAuto &&
            flightPhase.phase() ==
                FlightPhase::TARGET_CONFIRMED &&
            !guidedTransferFailed
        ) {
            const bool retryReady =
                !guidedRequestSent ||
                std::chrono::duration_cast<
                    std::chrono::milliseconds
                >(
                    now - lastGuidedRequest
                ).count() >= GUIDED_RETRY_MS;

            if (retryReady &&
                guidedAttempts < MAX_GUIDED_ATTEMPTS) {

                if (mavlink.requestGuided()) {
                    if (!guidedRequestSent)
                        firstGuidedRequest = now;

                    guidedRequestSent = true;
                    lastGuidedRequest = now;
                    ++guidedAttempts;

                    std::cout
                        << "[AUTO -> GUIDED] Attempt "
                        << guidedAttempts << "/"
                        << MAX_GUIDED_ATTEMPTS
                        << "\n";
                }
            }

            if (guidedRequestSent) {
                const auto waitingMs =
                    std::chrono::duration_cast<
                        std::chrono::milliseconds
                    >(
                        now - firstGuidedRequest
                    ).count();

                if (waitingMs >= GUIDED_TIMEOUT_MS) {
                    guidedTransferFailed = true;

                    std::cout
                        << "[AUTO -> GUIDED] "
                        << "TIMEOUT - no further retries\n";
                }
            }
        }

        // 5. Confirm through autopilot heartbeat.
        if (
            flightAuthorized &&
            guidedRequestSent &&
            !guidedTransferFailed &&
            phaseInput.inGuided &&
            !guidedTransferConfirmed
        ) {
            guidedTransferConfirmed = true;

            std::cout
                << "[AUTO -> GUIDED] "
                << "CONFIRMED by heartbeat\n";
        }

        const bool guidedControlAllowed =
            flightAuthorized &&
            guidedRequestSent &&
            guidedTransferConfirmed &&
            !guidedTransferFailed &&
            phaseInput.inGuided;

        // 6. Transmit BODY velocity (SITL only).
        if (
            guidedControlAllowed &&
            std::chrono::duration_cast<
                std::chrono::milliseconds
            >(
                now - lastVelocityTx
            ).count() >= VELOCITY_INTERVAL_MS
        ) {
            const PlannedMotion plan =
                flightPhase.plannedMotion(phaseInput);

            // Existing camera-to-BODY mapping.
            const float forward =
                plan.authorized
                    ? static_cast<float>(plan.imageX)
                    : 0.0f;

            const float right =
                plan.authorized
                    ? static_cast<float>(-plan.imageY)
                    : 0.0f;

            const float down =
                plan.authorized
                    ? static_cast<float>(plan.downMps)
                    : 0.0f;

            if (mavlink.sendBodyVelocity(
                forward,
                right,
                down
            )) {
                lastVelocityTx = now;
            }
        }

        // 7. Periodic debug output.
        if (
            std::chrono::duration_cast<
                std::chrono::milliseconds
            >(
                now - lastPlanLog
            ).count() >= 1000
        ) {
            const FlightPhase p =
                flightPhase.phase();

            if (
                p == FlightPhase::GUIDED_APPROACH ||
                p == FlightPhase::GUIDED_HOVER ||
                p == FlightPhase::GUIDED_DIAGONAL ||
                p == FlightPhase::GUIDED_STABILIZE ||
                p == FlightPhase::GUIDED_DESCEND ||
                p == FlightPhase::TARGET_LOST_HOVER ||
                p == FlightPhase::MISSION_ABORTED_HOVER ||
                p == FlightPhase::GUIDED_COMPLETE
            ) {
                const PlannedMotion plan =
                    flightPhase.plannedMotion(phaseInput);

                std::cout
                    << "[FLIGHT CONTROL] "
                    << (guidedControlAllowed
                        ? "SITL TX ENABLED"
                        : "NO TX")
                    << " | Forward="
                    << plan.imageX
                    << " | Right="
                    << -plan.imageY
                    << " | Down="
                    << plan.downMps
                    << " m/s\n";
            }

            lastPlanLog = now;
        }

        // 8. Console command input.
        pollfd input{};
        input.fd = 0;
        input.events = POLLIN;

        const int pollResult = poll(
            &input,
            1,
            100
        );

        if (pollResult <= 0 ||
            !(input.revents & POLLIN)) {
            continue;
        }

        std::string command;

        if (!std::getline(std::cin, command))
            break;

        std::transform(
            command.begin(),
            command.end(),
            command.begin(),
            [](unsigned char c) {
                return static_cast<char>(
                    std::toupper(c)
                );
            }
        );

        const auto start =
            command.find_first_not_of(" \t\r\n");

        if (start == std::string::npos)
            continue;

        const auto end =
            command.find_last_not_of(" \t\r\n");

        command = command.substr(
            start,
            end - start + 1
        );

        connected = mavlink.connected();
        health_report = health.check();

        if (command == "APPROACH VERTICAL" ||
            command == "APPROACH DIAGONAL") {

            // Algorithm selection is ground-only.
            if (mavlink.armed() ||
                mission.state() ==
                    MissionState::MISSION_ACTIVE) {

                std::cout
                    << "[APPROACH] DENIED: "
                    << "select before ARM / FLY\n";
                continue;
            }

            const ApproachMode requested =
                command == "APPROACH DIAGONAL"
                    ? ApproachMode::DIAGONAL
                    : ApproachMode::VERTICAL;

            if (!flightPhase.setApproachMode(requested)) {
                std::cout
                    << "[APPROACH] DENIED: "
                    << "flight phase not idle\n";
            }

        } else if (command == "FLY") {

            mission.fly(
                connected,
                health_report
            );

        } else if (command == "MANUAL") {

            mission.manualTakeover();
            guidedTransferConfirmed = false;

        } else if (command == "RESET") {

            mission.reset(
                connected,
                mavlink.armed(),
                health_report
            );

            if (!safety.isManualLocked() &&
                !mavlink.armed()) {

                guidedRequestSent = false;
                guidedTransferFailed = false;
                guidedTransferConfirmed = false;
                guidedAttempts = 0;
            }

        } else if (command == "STATUS") {

            mission.printStatus(
                connected,
                mavlink.armed(),
                mavlink.mode()
            );

            health.printStatus();
            mavlink.printAltitudeStatus();
            vision.printStatus();

            const PhaseInput currentInput =
                makePhaseInput(
                    mavlink,
                    vision,
                    safety,
                    mission
                );

            flightPhase.printStatus(currentInput);

            std::cout
                << "Guided TX   : "
                << (guidedTransferConfirmed
                    ? "CONFIRMED"
                    : "NOT CONFIRMED")
                << "\n"
                << "Guided tries: "
                << guidedAttempts
                << "\n"
                << "TX mode     : "
                << (config.enableFlightTx
                    ? "SITL ENABLED"
                    : "DISABLED")
                << "\n";

        } else if (command == "EXIT") {

            if (config.enableFlightTx &&
                guidedControlAllowed) {
                mavlink.sendBodyVelocity(
                    0.0f, 0.0f, 0.0f
                );
            }

            break;

        } else {
            std::cout
                << "[ERROR] Unknown command\n";
        }
    }

    std::cout
        << "[EXIT] Control stopped\n";

    return 0;
}
