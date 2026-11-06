#pragma once
#include <string>

class SafetyManager {
public:
    SafetyManager();

    void activateManualLockout();
    bool resetManualLockout(bool connected, bool armed);
    bool isManualLocked() const;
    bool canStartMission(bool connected) const;

private:
    bool saveState(const std::string& state);

    bool manual_lockout_ = true;
    std::string state_file_;
};
