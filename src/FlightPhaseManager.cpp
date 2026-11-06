
#include "FlightPhaseManager.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace {

const char* phaseName(FlightPhase p)
{
    switch (p) {
        case FlightPhase::IDLE:
            return "IDLE";

        case FlightPhase::AUTO_SEARCH:
            return "AUTO_SEARCH";

        case FlightPhase::TARGET_CONFIRMED:
            return "TARGET_CONFIRMED";

        case FlightPhase::GUIDED_APPROACH:
            return "GUIDED_APPROACH";

        case FlightPhase::GUIDED_HOVER:
            return "GUIDED_HOVER";

        case FlightPhase::GUIDED_DIAGONAL:
            return "GUIDED_DIAGONAL";

        case FlightPhase::GUIDED_STABILIZE:
            return "GUIDED_STABILIZE";

        case FlightPhase::GUIDED_DESCEND:
            return "GUIDED_DESCEND";

        case FlightPhase::TARGET_LOST_HOVER:
            return "TARGET_LOST_HOVER";

        case FlightPhase::MISSION_ABORTED_HOVER:
            return "MISSION_ABORTED_HOVER";

        case FlightPhase::GUIDED_COMPLETE:
            return "GUIDED_COMPLETE";

        case FlightPhase::MANUAL_LOCKOUT:
            return "MANUAL_LOCKOUT";
    }

    return "UNKNOWN";
}


const char* modeName(ApproachMode m)
{
    return m == ApproachMode::DIAGONAL
        ? "DIAGONAL (FIXED 45 DEG)"
        : "VERTICAL";
}


long elapsedMs(
    FlightPhaseManager::TimePoint now,
    FlightPhaseManager::TimePoint from
)
{
    return std::chrono::duration_cast<
        std::chrono::milliseconds
    >(now - from).count();
}

} // namespace


// ========================================
// Approach mode selection
// ========================================

bool FlightPhaseManager::setApproachMode(
    ApproachMode mode
)
{
    if (
        phase_ != FlightPhase::IDLE &&
        phase_ != FlightPhase::MANUAL_LOCKOUT
    ) {
        return false;
    }

    approachMode_ = mode;
    diagonalDirectionLocked_ = false;

    std::cout
        << "[APPROACH MODE] "
        << modeName(mode)
        << "\n";

    return true;
}


ApproachMode FlightPhaseManager::approachMode() const
{
    return approachMode_;
}


// ========================================
// Flight phase
// ========================================

void FlightPhaseManager::setPhase(
    FlightPhase next
)
{
    if (phase_ == next)
        return;

    phase_ = next;
    reacquiring_ = false;

    std::cout
        << "[FLIGHT PHASE] -> "
        << phaseName(phase_)
        << "\n";
}


FlightPhase FlightPhaseManager::phase() const
{
    return phase_;
}


// ========================================
// Target validation
// ========================================

bool FlightPhaseManager::targetUsable(
    const PhaseInput& in
) const
{
    return
        in.visionFresh &&
        in.targetVisible &&
        in.targetVerified &&
        in.imageWidth > 0 &&
        in.imageHeight > 0 &&
        in.targetCx >= 0 &&
        in.targetCx < in.imageWidth &&
        in.targetCy >= 0 &&
        in.targetCy < in.imageHeight;
}


bool FlightPhaseManager::withinError(
    const PhaseInput& in,
    double px
) const
{
    if (!targetUsable(in))
        return false;

    const double dx =
        in.targetCx - in.imageWidth / 2.0;

    const double dy =
        in.targetCy - in.imageHeight / 2.0;

    return
        std::abs(dx) <=
            px * in.imageWidth / 640.0 &&
        std::abs(dy) <=
            px * in.imageHeight / 480.0;
}


bool FlightPhaseManager::targetCentered(
    const PhaseInput& in
) const
{
    return withinError(in, 20.0);
}


// ========================================
// DIAGONAL: YOLO direction tracking
// ========================================

bool FlightPhaseManager::lockDiagonalDirection(
    const PhaseInput& in
)
{
    if (!targetUsable(in))
        return false;

    const double ex =
        (2.0 * in.targetCx /
         in.imageWidth) - 1.0;

    const double ey =
        (2.0 * in.targetCy /
         in.imageHeight) - 1.0;

    // Existing camera-to-BODY mapping.
    // main.cpp:
    // forward = plan.imageX
    // right   = -plan.imageY
    const double x = -ALIGN_KP * ey;
    const double y = -ALIGN_KP * ex;

    const double magnitude =
        std::hypot(x, y);

    // No reliable direction at exact center.
    // Temporarily keep the previous direction.
    if (magnitude < 1e-6)
        return diagonalDirectionLocked_;

    const bool firstLock =
        !diagonalDirectionLocked_;

    // Update from the latest YOLO coordinates.
    lockedImageX_ = x / magnitude;
    lockedImageY_ = y / magnitude;

    diagonalDirectionLocked_ = true;

    if (firstLock) {
        std::cout
            << "[DIAGONAL45] Tracking started"
            << " | Forward unit="
            << lockedImageX_
            << " | Right unit="
            << -lockedImageY_
            << "\n";
    }

    return true;
}


// ========================================
// Start approach
// ========================================

void FlightPhaseManager::startApproach(
    TimePoint now
)
{
    if (approachMode_ == ApproachMode::DIAGONAL) {

        diagonalDirectionLocked_ = false;

        hoverStarted_ = now;
        hoverStableTracking_ = false;

        setPhase(FlightPhase::GUIDED_HOVER);

    } else {

        // Keep original vertical realignment
        // behavior after a large tracking error.
        setPhase(FlightPhase::GUIDED_APPROACH);
    }
}


// ========================================
// AUTO search
// ========================================

bool FlightPhaseManager::startAutoSearch(
    const PhaseInput& in
)
{
    if (
        phase_ != FlightPhase::IDLE ||
        !in.connected ||
        in.manualLockout ||
        !in.missionActive ||
        !in.armed ||
        !in.inAuto
    ) {
        return false;
    }

    setPhase(FlightPhase::AUTO_SEARCH);
    return true;
}


// ========================================
// State machine
// ========================================

void FlightPhaseManager::update(
    const PhaseInput& in,
    TimePoint now
)
{
    // ------------------------------------
    // 1. Manual takeover
    // ------------------------------------

    if (in.manualLockout) {

        diagonalDirectionLocked_ = false;

        setPhase(
            FlightPhase::MANUAL_LOCKOUT
        );

        return;
    }

    if (phase_ == FlightPhase::MANUAL_LOCKOUT) {
        setPhase(FlightPhase::IDLE);
    }

    // ------------------------------------
    // 2. Mission authorization
    // ------------------------------------

    if (
        !in.connected ||
        !in.missionActive ||
        !in.armed
    ) {
        diagonalDirectionLocked_ = false;

        setPhase(FlightPhase::IDLE);
        return;
    }

    // ------------------------------------
    // 3. Phase transitions
    // ------------------------------------

    switch (phase_) {

    case FlightPhase::IDLE:
    case FlightPhase::MANUAL_LOCKOUT:
    case FlightPhase::GUIDED_COMPLETE:
        return;


    // ====================================
    // AUTO SEARCH
    // ====================================

    case FlightPhase::AUTO_SEARCH: {

        if (!in.inAuto) {
            setPhase(FlightPhase::IDLE);
            return;
        }

        if (
            !in.altitudeValid ||
            in.relativeAltitudeM <
                MIN_SEARCH_ALTITUDE_M
        ) {
            return;
        }

        if (targetUsable(in)) {
            setPhase(
                FlightPhase::TARGET_CONFIRMED
            );
        }

        return;
    }


    // ====================================
    // TARGET CONFIRMED
    // ====================================

    case FlightPhase::TARGET_CONFIRMED: {

        // Continue AUTO until GUIDED is
        // confirmed by autopilot heartbeat.
        if (in.inAuto)
            return;

        if (!in.inGuided) {
            setPhase(FlightPhase::IDLE);
            return;
        }

        if (
            !in.altitudeValid ||
            !targetUsable(in)
        ) {
            lostSince_ = now;

            setPhase(
                FlightPhase::TARGET_LOST_HOVER
            );

            return;
        }

        // ==================================
        // CHANGED:
        // BOTH algorithms now start
        // with GUIDED_HOVER.
        // ==================================

        diagonalDirectionLocked_ = false;

        hoverStarted_ = now;
        hoverStableTracking_ = false;

        setPhase(
            FlightPhase::GUIDED_HOVER
        );

        return;
    }


    // ====================================
    // GUIDED HOVER
    // BOTH VERTICAL AND DIAGONAL
    // ====================================

    case FlightPhase::GUIDED_HOVER: {

        if (!in.inGuided) {
            setPhase(FlightPhase::IDLE);
            return;
        }

        if (!in.altitudeValid) {

            setPhase(
                FlightPhase::MISSION_ABORTED_HOVER
            );

            return;
        }

        if (!targetUsable(in)) {

            lostSince_ = now;

            setPhase(
                FlightPhase::TARGET_LOST_HOVER
            );

            return;
        }

        // Actual velocity must be close
        // to zero before the 2s timer runs.
        //
        // Horizontal speed: <= 0.08 m/s
        // Vertical speed:   <= 0.08 m/s

        const bool settled =
            in.velocityValid &&
            in.horizontalSpeedMps <=
                HOVER_SPEED_TOLERANCE_MPS &&
            std::abs(in.verticalSpeedMps) <=
                HOVER_SPEED_TOLERANCE_MPS;

        if (settled) {

            if (!hoverStableTracking_) {

                hoverStableTracking_ = true;
                hoverStableSince_ = now;

            } else if (
                elapsedMs(
                    now,
                    hoverStableSince_
                ) >= STABILIZE_MS
            ) {

                if (
                    in.relativeAltitudeM <=
                        STOP_ALTITUDE_M
                ) {
                    setPhase(
                        FlightPhase::GUIDED_COMPLETE
                    );
                    return;
                }

                // ==========================
                // VERTICAL:
                // Start horizontal alignment
                // after hovering.
                // ==========================

                if (
                    approachMode_ ==
                        ApproachMode::VERTICAL
                ) {

                    std::cout
                        << "[HOVER] Stable for 2s"
                        << " -> VERTICAL alignment\n";

                    setPhase(
                        FlightPhase::GUIDED_APPROACH
                    );

                    return;
                }

                // ==========================
                // DIAGONAL:
                // Start fixed 45-degree
                // descent after hovering.
                // ==========================

                if (!lockDiagonalDirection(in)) {

                    std::cout
                        << "[DIAGONAL45] No valid "
                        << "starting direction; abort\n";

                    setPhase(
                        FlightPhase::MISSION_ABORTED_HOVER
                    );

                    return;
                }

                std::cout
                    << "[HOVER] Stable for 2s"
                    << " -> DIAGONAL45\n";

                setPhase(
                    FlightPhase::GUIDED_DIAGONAL
                );

                return;
            }

        } else {

            // Restart the timer if movement
            // exceeds the velocity tolerance.
            hoverStableTracking_ = false;
        }

        if (
            elapsedMs(
                now,
                hoverStarted_
            ) >= HOVER_TIMEOUT_MS
        ) {

            std::cout
                << "[HOVER] Stabilization timeout\n";

            setPhase(
                FlightPhase::MISSION_ABORTED_HOVER
            );
        }

        return;
    }


    // ====================================
    // VERTICAL: INITIAL ALIGNMENT
    // ====================================

    case FlightPhase::GUIDED_APPROACH: {

        if (!in.inGuided) {
            setPhase(FlightPhase::IDLE);
            return;
        }

        if (!in.altitudeValid) {

            setPhase(
                FlightPhase::MISSION_ABORTED_HOVER
            );

            return;
        }

        if (!targetUsable(in)) {

            lostSince_ = now;

            setPhase(
                FlightPhase::TARGET_LOST_HOVER
            );

            return;
        }

        if (targetCentered(in)) {

            stableSince_ = now;

            setPhase(
                FlightPhase::GUIDED_STABILIZE
            );
        }

        return;
    }


    // ====================================
    // DIAGONAL: 45-DEGREE DESCENT
    // ====================================

    case FlightPhase::GUIDED_DIAGONAL: {

        if (!in.inGuided) {

            diagonalDirectionLocked_ = false;
            setPhase(FlightPhase::IDLE);
            return;
        }

        if (
            !in.altitudeValid ||
            !in.velocityValid ||
            !diagonalDirectionLocked_
        ) {

            setPhase(
                FlightPhase::MISSION_ABORTED_HOVER
            );

            return;
        }

        if (!targetUsable(in)) {

            lostSince_ = now;

            setPhase(
                FlightPhase::TARGET_LOST_HOVER
            );

            return;
        }

        if (
            in.relativeAltitudeM <=
                STOP_ALTITUDE_M
        ) {

            setPhase(
                FlightPhase::GUIDED_COMPLETE
            );

            return;
        }

        // Continuously track the basket.
        // The horizontal direction changes,
        // but horizontal and vertical speed
        // magnitudes remain equal (0.2 m/s).

        if (!lockDiagonalDirection(in)) {

            setPhase(
                FlightPhase::MISSION_ABORTED_HOVER
            );

            return;
        }

        // No automatic transition to
        // GUIDED_DESCEND in DIAGONAL mode.

        return;
    }


    // ====================================
    // VERTICAL: STABILIZATION
    // ====================================

    case FlightPhase::GUIDED_STABILIZE: {

        if (!in.inGuided) {
            setPhase(FlightPhase::IDLE);
            return;
        }

        if (!in.altitudeValid) {

            setPhase(
                FlightPhase::MISSION_ABORTED_HOVER
            );

            return;
        }

        if (!targetUsable(in)) {

            lostSince_ = now;

            setPhase(
                FlightPhase::TARGET_LOST_HOVER
            );

            return;
        }

        if (!targetCentered(in)) {

            setPhase(
                FlightPhase::GUIDED_APPROACH
            );

            return;
        }

        if (
            elapsedMs(
                now,
                stableSince_
            ) >= STABILIZE_MS
        ) {

            setPhase(
                in.relativeAltitudeM <=
                    STOP_ALTITUDE_M
                    ? FlightPhase::GUIDED_COMPLETE
                    : FlightPhase::GUIDED_DESCEND
            );
        }

        return;
    }


    // ====================================
    // VERTICAL: DESCENT
    // ====================================

    case FlightPhase::GUIDED_DESCEND: {

        if (!in.inGuided) {
            setPhase(FlightPhase::IDLE);
            return;
        }

        if (!in.altitudeValid) {

            setPhase(
                FlightPhase::MISSION_ABORTED_HOVER
            );

            return;
        }

        if (!targetUsable(in)) {

            lostSince_ = now;

            setPhase(
                FlightPhase::TARGET_LOST_HOVER
            );

            return;
        }

        if (
            in.relativeAltitudeM <=
                STOP_ALTITUDE_M
        ) {

            setPhase(
                FlightPhase::GUIDED_COMPLETE
            );

            return;
        }

        if (!withinError(in, 60.0)) {

            std::cout
                << "[DESCEND] Large target error: "
                << "realign\n";

            // Keep the original behavior:
            // return to horizontal alignment.
            startApproach(now);
        }

        return;
    }


    // ====================================
    // TARGET LOST
    // ====================================

    case FlightPhase::TARGET_LOST_HOVER: {

        if (!in.inGuided) {
            setPhase(FlightPhase::IDLE);
            return;
        }

        if (
            elapsedMs(
                now,
                lostSince_
            ) >= LOST_ABORT_MS
        ) {

            setPhase(
                FlightPhase::MISSION_ABORTED_HOVER
            );

            return;
        }

        if (
            !targetUsable(in) ||
            !in.altitudeValid
        ) {

            reacquiring_ = false;
            return;
        }

        if (!reacquiring_) {

            reacquiring_ = true;
            reacquiredSince_ = now;

            return;
        }

        if (
            elapsedMs(
                now,
                reacquiredSince_
            ) >= REACQUIRE_MS
        ) {

            // Reacquire using the selected mode.
            startApproach(now);
        }

        return;
    }


    // ====================================
    // MISSION ABORT
    // ====================================

    case FlightPhase::MISSION_ABORTED_HOVER: {

        if (!in.inGuided)
            setPhase(FlightPhase::IDLE);

        return;
    }

    }
}


// ========================================
// Velocity calculation
// ========================================

PlannedMotion FlightPhaseManager::plannedMotion(
    const PhaseInput& in
) const
{
    PlannedMotion plan{};

    if (
        !in.connected ||
        in.manualLockout ||
        !in.missionActive ||
        !in.armed ||
        !in.inGuided ||
        !in.altitudeValid
    ) {
        return plan;
    }

    // Hover and abort phases request
    // zero BODY velocity.
    if (
        phase_ == FlightPhase::GUIDED_HOVER ||
        phase_ == FlightPhase::TARGET_LOST_HOVER ||
        phase_ == FlightPhase::MISSION_ABORTED_HOVER ||
        phase_ == FlightPhase::GUIDED_COMPLETE
    ) {

        plan.authorized = true;
        return plan;
    }

    if (
        !targetUsable(in) ||
        in.relativeAltitudeM <=
            STOP_ALTITUDE_M
    ) {
        return plan;
    }

    // Vertical stabilization:
    // zero horizontal and vertical velocity.
    if (
        phase_ == FlightPhase::GUIDED_STABILIZE
    ) {

        plan.authorized = true;
        return plan;
    }


    // ====================================
    // DIAGONAL: FIXED 45-DEGREE SPEEDS
    // ====================================

    if (
        phase_ == FlightPhase::GUIDED_DIAGONAL
    ) {

        if (
            !in.velocityValid ||
            !diagonalDirectionLocked_
        ) {
            return plan;
        }

        // Unit horizontal direction is
        // continuously updated by YOLO.
        //
        // sqrt(imageX^2 + imageY^2) = 0.2
        // downMps = 0.2
        //
        // Commanded descent angle = 45 deg.

        plan.imageX =
            lockedImageX_ * MAX_SPEED_MPS;

        plan.imageY =
            lockedImageY_ * MAX_SPEED_MPS;

        plan.downMps =
            MAX_SPEED_MPS;

        plan.authorized = true;
        return plan;
    }


    // ====================================
    // VERTICAL: ORIGINAL ALGORITHM
    // ====================================

    const bool initialVertical =
        phase_ == FlightPhase::GUIDED_APPROACH;

    const bool verticalDescending =
        phase_ == FlightPhase::GUIDED_DESCEND;

    if (
        !initialVertical &&
        !verticalDescending
    ) {
        return plan;
    }

    const double ex =
        (2.0 * in.targetCx /
         in.imageWidth) - 1.0;

    const double ey =
        (2.0 * in.targetCy /
         in.imageHeight) - 1.0;

    const double rawX =
        -ALIGN_KP * ey;

    const double rawY =
        -ALIGN_KP * ex;

    const double dx =
        in.targetCx - in.imageWidth / 2.0;

    const double dy =
        in.targetCy - in.imageHeight / 2.0;

    const double x =
        std::abs(dy) <=
            4.0 * in.imageHeight / 480.0
                ? 0.0 : rawX;

    const double y =
        std::abs(dx) <=
            4.0 * in.imageWidth / 640.0
                ? 0.0 : rawY;

    const double magnitude =
        std::hypot(x, y);

    const double speedLimit =
        verticalDescending
            ? DESCEND_CORRECTION_SPEED_MPS
            : MAX_SPEED_MPS;

    if (magnitude > speedLimit) {

        const double scale =
            speedLimit / magnitude;

        plan.imageX = x * scale;
        plan.imageY = y * scale;

    } else {

        plan.imageX = x;
        plan.imageY = y;
    }

    plan.authorized = true;

    if (verticalDescending) {

        // Preserve existing vertical descent:
        // below 3m, descend only if centered.
        if (
            in.relativeAltitudeM > 3.0 ||
            targetCentered(in)
        ) {

            plan.downMps =
                MAX_SPEED_MPS;
        }
    }

    return plan;
}


// ========================================
// Hover request
// ========================================

bool FlightPhaseManager::hoverRequested(
    const PhaseInput& in
) const
{
    if (
        !in.connected ||
        in.manualLockout ||
        !in.missionActive ||
        !in.armed ||
        !in.inGuided
    ) {
        return false;
    }

    return
        phase_ == FlightPhase::GUIDED_HOVER ||
        phase_ == FlightPhase::TARGET_LOST_HOVER ||
        phase_ == FlightPhase::MISSION_ABORTED_HOVER;
}


// ========================================
// Status output
// ========================================

void FlightPhaseManager::printStatus(
    const PhaseInput& in
) const
{
    std::cout
        << "\n===== FLIGHT PHASE =====\n"
        << "Approach    : "
        << modeName(approachMode_)
        << "\n"
        << "Phase       : "
        << phaseName(phase_)
        << "\n"
        << "Mission     : "
        << (in.missionActive
            ? "ACTIVE" : "INACTIVE")
        << "\n"
        << "Armed       : "
        << (in.armed
            ? "YES" : "NO")
        << "\n";

    if (in.altitudeValid) {

        std::cout
            << "Altitude    : "
            << in.relativeAltitudeM
            << " m\n";

    } else {

        std::cout
            << "Altitude    : NO DATA\n";
    }

    if (in.velocityValid) {

        std::cout
            << "Horiz speed : "
            << in.horizontalSpeedMps
            << " m/s\n"
            << "Vert speed  : "
            << in.verticalSpeedMps
            << " m/s\n";

    } else {

        std::cout
            << "Speed       : NO DATA\n";
    }

    std::cout << "Target      : ";

    if (!in.visionFresh) {

        std::cout << "NO DATA\n";

    } else if (in.targetVerified) {

        std::cout << "VERIFIED\n";

    } else if (in.targetVisible) {

        std::cout << "UNVERIFIED\n";

    } else {

        std::cout << "NOT DETECTED\n";
    }

    if (targetUsable(in)) {

        std::cout
            << "Target pixel: ("
            << in.targetCx << ", "
            << in.targetCy << ")\n"
            << "Center error: ("
            << in.targetCx -
                in.imageWidth / 2.0
            << ", "
            << in.targetCy -
                in.imageHeight / 2.0
            << ") px\n"
            << "Centered    : "
            << (targetCentered(in)
                ? "YES" : "NO")
            << "\n";
    }

    if (
        approachMode_ == ApproachMode::DIAGONAL
    ) {

        std::cout
            << "Direction   : "
            << (diagonalDirectionLocked_
                ? "TRACKING" : "NOT READY")
            << "\n";
    }

    const PlannedMotion p =
        plannedMotion(in);

    std::cout
        << std::fixed
        << std::setprecision(3)
        << "Plan image X: "
        << p.imageX << " m/s\n"
        << "Plan image Y: "
        << p.imageY << " m/s\n"
        << "Plan down   : "
        << p.downMps << " m/s\n"
        << "Plan valid  : "
        << (p.authorized
            ? "YES" : "NO")
        << "\n"
        << "Hover Req   : "
        << (hoverRequested(in)
            ? "YES" : "NO")
        << "\n"
        << "Flight TX   : CONTROLLED BY MAIN\n"
        << "========================\n";

    std::cout.unsetf(
        std::ios::floatfield
    );
}
