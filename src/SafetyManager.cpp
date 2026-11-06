#include "SafetyManager.hpp"

#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>

namespace fs = std::filesystem;

SafetyManager::SafetyManager() {
    const char* home = std::getenv("HOME");
    fs::path base = home ? fs::path(home) : fs::path(".");

    state_file_ =
        (base / "control/state/manual_lockout.state").string();

    std::ifstream file(state_file_);
    std::string previous;

    if (file >> previous) {
        std::cout << "[SAFETY] Previous state: "
                  << previous << "\n";
    } else {
        std::cout << "[SAFETY] No previous state\n";
    }

    // Always start locked.
    manual_lockout_ = true;

    if (!saveState("LOCKED")) {
        std::cerr << "[SAFETY] State save failed\n";
    }

    std::cout << "[SAFETY] Startup Lockout ACTIVE\n";
}

bool SafetyManager::saveState(const std::string& state) {
    fs::path path(state_file_);
    std::error_code ec;

    fs::create_directories(path.parent_path(), ec);
    if (ec) return false;

    std::string temp = state_file_ + ".tmp";

    int fd = open(temp.c_str(),
                  O_WRONLY | O_CREAT | O_TRUNC,
                  0600);

    if (fd < 0) return false;

    std::string data = state + "\n";
    size_t offset = 0;
    bool ok = true;

    while (offset < data.size()) {
        ssize_t n = write(fd,
                          data.data() + offset,
                          data.size() - offset);

        if (n < 0 && errno == EINTR) continue;

        if (n <= 0) {
            ok = false;
            break;
        }

        offset += static_cast<size_t>(n);
    }

    if (ok && fsync(fd) != 0) ok = false;
    if (close(fd) != 0) ok = false;

    if (ok && rename(temp.c_str(),
                     state_file_.c_str()) != 0) {
        ok = false;
    }

    if (!ok) {
        unlink(temp.c_str());
        return false;
    }

    int dirfd = open(path.parent_path().c_str(),
                     O_RDONLY | O_DIRECTORY);

    if (dirfd < 0) return false;

    bool synced = (fsync(dirfd) == 0);
    if (close(dirfd) != 0) synced = false;

    return synced;
}

void SafetyManager::activateManualLockout() {
    manual_lockout_ = true;

    if (!saveState("LOCKED")) {
        std::cerr << "[SAFETY] Lockout save failed\n";
    }

    std::cout << "[SAFETY] MANUAL_LOCKOUT activated\n";
}

bool SafetyManager::resetManualLockout(
    bool connected, bool armed) {

    if (!connected || armed) {
        std::cout << "[SAFETY] RESET denied: "
                     "connected and disarmed required\n";
        return false;
    }

    if (!saveState("UNLOCKED")) {
        std::cout << "[SAFETY] RESET denied: "
                     "state save failed\n";
        return false;
    }

    manual_lockout_ = false;
    std::cout << "[SAFETY] Lockout reset\n";
    return true;
}

bool SafetyManager::isManualLocked() const {
    return manual_lockout_;
}

bool SafetyManager::canStartMission(bool connected) const {
    return connected && !manual_lockout_;
}
