
#pragma once


#pragma once

#include <cstdint>

#include "SafetyManager.hpp"
#include "HealthMonitor.hpp"

enum class MissionState {
    HEALTH_CHECK,
    READY,
    MISSION_ACTIVE,
    MANUAL_LOCKOUT
};

class MissionManager {
public:
    explicit MissionManager(SafetyManager& safety);

    void update(
        bool connected,
        const HealthReport& health
    );

    void fly(
        bool connected,
        const HealthReport& health
    );

    void manualTakeover();

    void reset(
        bool connected,
        bool armed,
        const HealthReport& health
    );

    void onModeChanged(
        uint32_t oldMode,
        uint32_t newMode
    );

    void printStatus(
        bool connected,
        bool armed,
        uint32_t mode
    ) const;

    // Read current mission state.
    MissionState state() const {
        return state_;
    }

private:
    void setState(MissionState next);
    const char* stateName() const;

    SafetyManager& safety_;

    MissionState state_ = MissionState::HEALTH_CHECK;
};
