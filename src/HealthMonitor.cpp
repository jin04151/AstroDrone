
#include "HealthMonitor.hpp"

#include <iostream>
#include <iomanip>
#include <limits>

HealthMonitor::HealthMonitor(
    std::chrono::milliseconds telemetryTimeout
)
    : telemetry_timeout_(telemetryTimeout) {}

bool HealthMonitor::isFresh(
    bool received,
    Clock::time_point updated
) const
{
    return received &&
           Clock::now() - updated <
               telemetry_timeout_;
}

const char* HealthMonitor::statusName(
    CheckStatus status
)
{
    switch (status) {
        case CheckStatus::PASS:
            return "PASS";
        case CheckStatus::FAIL:
            return "FAIL";
        case CheckStatus::NO_DATA:
            return "NO DATA";
    }

    return "UNKNOWN";
}

void HealthMonitor::processMessage(
    const mavlink_message_t& msg
)
{
    auto now = Clock::now();

    switch (msg.msgid) {

    case MAVLINK_MSG_ID_SYS_STATUS: {

        mavlink_sys_status_t data{};
        mavlink_msg_sys_status_decode(&msg, &data);

        battery_.received = true;
        battery_.updated = now;

        battery_.voltage =
            data.voltage_battery == UINT16_MAX
            ? -1.0
            : data.voltage_battery / 1000.0;

        battery_.remaining = data.battery_remaining;

        break;
    }

    case MAVLINK_MSG_ID_BATTERY_STATUS: {

        mavlink_battery_status_t data{};
        mavlink_msg_battery_status_decode(&msg, &data);

        if (data.id != 0)
            break;

        battery_.received = true;
        battery_.updated = now;

        uint32_t total_mv = 0;
        bool valid = false;

        for (int i = 0; i < 10; ++i) {

            uint16_t mv = data.voltages[i];

            if (mv == UINT16_MAX || mv == 0)
                continue;

            total_mv += mv;
            valid = true;
        }

        battery_.voltage =
            valid ? total_mv / 1000.0 : -1.0;

        battery_.remaining = data.battery_remaining;

        break;
    }

    case MAVLINK_MSG_ID_GPS_RAW_INT: {

        mavlink_gps_raw_int_t data{};
        mavlink_msg_gps_raw_int_decode(&msg, &data);

        gps_.received = true;
        gps_.updated = now;

        gps_.fix = data.fix_type;
        gps_.satellites = data.satellites_visible;

        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS: {

        mavlink_rc_channels_t data{};
        mavlink_msg_rc_channels_decode(&msg, &data);

        rc_.received = true;
        rc_.updated = now;

        rc_.channels = data.chancount;
        rc_.rssi = data.rssi;

        break;
    }

    default:
        break;
    }
}

HealthReport HealthMonitor::check() const
{
    HealthReport report{};

    // Battery
    if (!isFresh(battery_.received, battery_.updated) ||
        battery_.remaining < 0 ||
        battery_.remaining > 100 ||
        battery_.voltage <= 0.0) {

        report.battery = CheckStatus::NO_DATA;

    } else if (
        battery_.remaining < MIN_BATTERY_PERCENT
    ) {

        report.battery = CheckStatus::FAIL;

    } else {

        report.battery = CheckStatus::PASS;
    }

    // GPS
    if (!isFresh(gps_.received, gps_.updated) ||
        gps_.satellites == 255) {

        report.gps = CheckStatus::NO_DATA;

    } else if (
        gps_.fix < MIN_GPS_FIX ||
        gps_.satellites < MIN_SATELLITES
    ) {

        report.gps = CheckStatus::FAIL;

    } else {

        report.gps = CheckStatus::PASS;
    }

    // RC telemetry
    if (!isFresh(rc_.received, rc_.updated)) {

        report.rc = CheckStatus::NO_DATA;

    } else if (rc_.channels <= 0) {

        report.rc = CheckStatus::FAIL;

    } else {

        report.rc = CheckStatus::PASS;
    }

    report.ready =
        report.battery == CheckStatus::PASS &&
        report.gps == CheckStatus::PASS &&
        report.rc == CheckStatus::PASS;

    return report;
}

void HealthMonitor::printStatus() const
{
    HealthReport report = check();

    std::cout
        << "\n===== HEALTH MONITOR =====\n";

    // Battery
    std::cout << "Battery : ";

    if (!isFresh(
        battery_.received,
        battery_.updated
    )) {

        std::cout << "NO DATA / STALE";

    } else {

        if (battery_.voltage >= 0.0) {

            std::cout
                << std::fixed
                << std::setprecision(2)
                << battery_.voltage << " V";

        } else {

            std::cout << "Voltage N/A";
        }

        if (battery_.remaining >= 0 &&
            battery_.remaining <= 100) {

            std::cout
                << " | "
                << battery_.remaining << "%";

        } else {

            std::cout << " | Remaining N/A";
        }
    }

    std::cout
        << " [" << statusName(report.battery)
        << "]\n";

    // GPS
    std::cout << "GPS     : ";

    if (!isFresh(gps_.received, gps_.updated)) {

        std::cout << "NO DATA / STALE";

    } else {

        std::cout
            << "Fix " << gps_.fix
            << " | Satellites ";

        if (gps_.satellites == 255)
            std::cout << "N/A";
        else
            std::cout << gps_.satellites;
    }

    std::cout
        << " [" << statusName(report.gps)
        << "]\n";

    // RC
    std::cout << "RC      : ";

    if (!isFresh(rc_.received, rc_.updated)) {

        std::cout << "NO DATA / STALE";

    } else {

        std::cout
            << "Channels " << rc_.channels
            << " | RSSI ";

        if (rc_.rssi == 255)
            std::cout << "N/A";
        else
            std::cout << rc_.rssi;
    }

    std::cout
        << " [" << statusName(report.rc)
        << "]\n";

    std::cout
        << "Telemetry: "
        << (report.ready ? "PASS" : "NOT READY")
        << "\n";

    std::cout
        << "Flight TX : DISABLED\n";

    std::cout
        << "==========================\n";
}
