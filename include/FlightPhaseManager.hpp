
#pragma once

#include <chrono>

enum class ApproachMode {
    VERTICAL,
    DIAGONAL
};

enum class FlightPhase {
    IDLE,
    AUTO_SEARCH,
    TARGET_CONFIRMED,
    GUIDED_APPROACH,
    GUIDED_HOVER,
    GUIDED_DIAGONAL,
    GUIDED_STABILIZE,
    GUIDED_DESCEND,
    TARGET_LOST_HOVER,
    MISSION_ABORTED_HOVER,
    GUIDED_COMPLETE,
    MANUAL_LOCKOUT
};

struct PhaseInput {
    bool connected = false;
    bool manualLockout = true;
    bool missionActive = false;
    bool armed = false;

    bool inAuto = false;
    bool inGuided = false;

    bool altitudeValid = false;
    double relativeAltitudeM = 0.0;

    bool velocityValid = false;
    double horizontalSpeedMps = 0.0;
    double verticalSpeedMps = 0.0;

    bool targetVerified = false;
    bool targetVisible = false;
    bool visionFresh = false;

    int targetCx = 0;
    int targetCy = 0;
    int imageWidth = 0;
    int imageHeight = 0;
};

struct PlannedMotion {
    double imageX = 0.0;
    double imageY = 0.0;

    // Positive = downward (NED)
    double downMps = 0.0;

    bool authorized = false;
};

class FlightPhaseManager {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    bool setApproachMode(ApproachMode mode);
    ApproachMode approachMode() const;

    bool startAutoSearch(const PhaseInput& input);

    void update(
        const PhaseInput& input,
        TimePoint now = Clock::now()
    );

    FlightPhase phase() const;

    PlannedMotion plannedMotion(
        const PhaseInput& input
    ) const;

    bool hoverRequested(
        const PhaseInput& input
    ) const;

    void printStatus(
        const PhaseInput& input
    ) const;

private:
    void setPhase(FlightPhase next);

    bool targetUsable(
        const PhaseInput& input
    ) const;

    bool targetCentered(
        const PhaseInput& input
    ) const;

    bool withinError(
        const PhaseInput& input,
        double pixelLimit
    ) const;

    void startApproach(TimePoint now);

    // Lock the horizontal direction once,
    // immediately before fixed-45 descent.
    bool lockDiagonalDirection(
        const PhaseInput& input
    );

    FlightPhase phase_ = FlightPhase::IDLE;

    ApproachMode approachMode_ =
        ApproachMode::VERTICAL;

    TimePoint stableSince_{};

    TimePoint hoverStarted_{};
    TimePoint hoverStableSince_{};
    bool hoverStableTracking_ = false;

    TimePoint lostSince_{};
    TimePoint reacquiredSince_{};
    bool reacquiring_ = false;

    // Horizontal direction for 45-degree descent.
    // Image-space vector: unit length.
    double lockedImageX_ = 0.0;
    double lockedImageY_ = 0.0;
    bool diagonalDirectionLocked_ = false;

    static constexpr double MIN_SEARCH_ALTITUDE_M = 3.0;

    // Valid for current flat-ground SITL world.
    static constexpr double STOP_ALTITUDE_M = 1.0;

    static constexpr double MAX_SPEED_MPS = 0.2;
    static constexpr double DESCEND_CORRECTION_SPEED_MPS = 0.1;
    static constexpr double ALIGN_KP = 0.5;

    static constexpr double HOVER_SPEED_TOLERANCE_MPS = 0.08;

    static constexpr int STABILIZE_MS = 2000;
    static constexpr int HOVER_TIMEOUT_MS = 15000;
    static constexpr int REACQUIRE_MS = 1000;
    static constexpr int LOST_ABORT_MS = 5000;
};
