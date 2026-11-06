
#include "MissionManager.hpp"

#include <iostream>

MissionManager::MissionManager(SafetyManager& safety)
    : safety_(safety) {}

// Convert mission state to string
const char* MissionManager::stateName() const {
    switch (state_) {
        case MissionState::HEALTH_CHECK:
            return "HEALTH_CHECK";

        case MissionState::READY:
            return "READY";

        case MissionState::MISSION_ACTIVE:
            return "MISSION_ACTIVE";

        case MissionState::MANUAL_LOCKOUT:
            return "MANUAL_LOCKOUT";
    }

    return "UNKNOWN";
}

// Change mission state
void MissionManager::setState(MissionState next) {
    if (state_ == next)
        return;

    state_ = next;

    std::cout << "[MISSION] State -> "
              << stateName() << "\n";
}

// Periodic mission state update
void MissionManager::update(
    bool connected,
    const HealthReport& health)
{
    // 1. Manual lockout has highest priority
    if (safety_.isManualLocked()) {
        setState(MissionState::MANUAL_LOCKOUT);
        return;
    }

    // 2. Heartbeat connection check
    if (!connected) {
        if (state_ == MissionState::MISSION_ACTIVE) {
            std::cout
                << "[MISSION] Connection lost: "
                << "mission suspended\n";
        }

        // Communication loss is not manual takeover.
        // A new FLY command is required after recovery.
        setState(MissionState::HEALTH_CHECK);
        return;
    }

    // 3. Preliminary health check
    if (!health.ready) {
        if (state_ == MissionState::MISSION_ACTIVE) {
            std::cout
                << "[MISSION] Health check failed: "
                << "mission suspended\n";
        }

        setState(MissionState::HEALTH_CHECK);
        return;
    }

    // 4. Health check passed
    if (state_ == MissionState::HEALTH_CHECK) {
        setState(MissionState::READY);

        std::cout
            << "[MISSION] Preliminary health check passed\n"
            << "[MISSION] Waiting for FLY command\n";
    }
}

// Start mission (DRY RUN)
void MissionManager::fly(
    bool connected,
    const HealthReport& health)
{
    if (state_ != MissionState::READY) {
        std::cout
            << "[MISSION] FLY denied: "
            << "system is not READY\n";
        return;
    }

    if (!safety_.canStartMission(connected)) {
        std::cout
            << "[MISSION] FLY denied: "
            << "safety lockout or connection failure\n";
        return;
    }

    if (!health.ready) {
        std::cout
            << "[MISSION] FLY denied: "
            << "health check failed\n";
        return;
    }

    setState(MissionState::MISSION_ACTIVE);

    std::cout
        << "[MISSION] DRY RUN: "
        << "no MAVLink flight command sent\n";
}

// Manual takeover
void MissionManager::manualTakeover() {
    if (!safety_.isManualLocked()) {
        safety_.activateManualLockout();
    }

    setState(MissionState::MANUAL_LOCKOUT);
}

// Reset manual lockout
void MissionManager::reset(
    bool connected,
    bool armed,
    const HealthReport& health)
{
    if (!safety_.resetManualLockout(connected, armed)) {
        return;
    }

    setState(MissionState::HEALTH_CHECK);

    // Check health again after reset
    update(connected, health);
}

// Detect flight mode changes
void MissionManager::onModeChanged(
    uint32_t oldMode,
    uint32_t newMode)
{
    constexpr uint32_t AUTO = 3;
    constexpr uint32_t GUIDED = 4;

    std::cout
        << "[MAVLINK] Mode changed: "
        << oldMode << " -> " << newMode << "\n";

    // Only monitor takeover during active mission
    if (state_ != MissionState::MISSION_ACTIVE)
        return;

    bool wasAutomatic =
        oldMode == AUTO || oldMode == GUIDED;

    bool leftAutomatic =
        newMode != AUTO && newMode != GUIDED;

    if (wasAutomatic && leftAutomatic) {
        std::cout
            << "[SAFETY] Automatic mode interrupted\n";

        manualTakeover();
    }
}

// Print mission status
void MissionManager::printStatus(
    bool connected,
    bool armed,
    uint32_t mode) const
{
    std::cout << "\n===== CONTROL STATUS =====\n";

    std::cout << "Connection : "
              << (connected ? "OK" : "LOST") << "\n";

    std::cout << "State      : "
              << stateName() << "\n";

    std::cout << "Mode       : "
              << mode << "\n";

    std::cout << "Armed      : "
              << (armed ? "YES" : "NO") << "\n";

    std::cout << "Lockout    : "
              << (safety_.isManualLocked()
                  ? "ACTIVE" : "INACTIVE") << "\n";

    std::cout << "Flight TX  : DISABLED\n";

    std::cout << "==========================\n";
}

