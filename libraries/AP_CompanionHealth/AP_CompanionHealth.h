/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "AP_CompanionHealth_config.h"

#if AP_COMPANION_HEALTH_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_CompanionHealth {
public:
    AP_CompanionHealth();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_CompanionHealth);

    // health states (matches companion-side enum)
    enum class State : uint8_t {
        DISCONNECTED = 0,   // no messages or timed out
        HEALTHY = 1,        // all metrics normal
        DEGRADED = 2,       // warning thresholds exceeded
        CRITICAL = 3,       // critical thresholds exceeded
    };

    // get singleton instance
    static AP_CompanionHealth *get_singleton() {
        return _singleton;
    }

    // called when COMPANION_HEALTH message is received
    void handle_message(const mavlink_message_t &msg);

    // check for timeout, called from the vehicle's 3Hz loop
    void update();

    // accessors for failsafe state
    // returns true only for HEALTHY or DEGRADED, false for DISCONNECTED or CRITICAL
    bool is_healthy() const { return _state == State::HEALTHY || _state == State::DEGRADED; }
    bool has_ever_connected() const { return _last_msg_ms > 0; }
    State get_state() const { return _state; }
    const char* get_state_name() const;
    uint32_t last_message_age_ms() const;

    // get failsafe enable parameter value (same values as GCS failsafe)
    int8_t get_failsafe_action() const { return _fs_enable; }

    // latest companion status (for logging and GCS display)
    struct CompanionStatus {
        uint32_t services_status;
        uint16_t watchdog_seq;
        int16_t temperature;      // cdegC, INT16_MAX if unknown
        uint8_t cpu_load;         // 0-100%
        uint8_t memory_used;      // 0-100%
        uint8_t disk_used;        // 0-100%
        uint8_t gpu_load;         // 0-100%, 255 if N/A
        uint8_t status_flags;     // bit flags
    };
    const CompanionStatus& get_status() const { return _status; }

    // status flag bits
    static constexpr uint8_t STATUS_FLAG_THROTTLED   = (1 << 0);
    static constexpr uint8_t STATUS_FLAG_OVERHEATING = (1 << 1);
    static constexpr uint8_t STATUS_FLAG_LOW_MEMORY  = (1 << 2);
    static constexpr uint8_t STATUS_FLAG_LOW_DISK    = (1 << 3);
    static constexpr uint8_t STATUS_FLAG_ALL = STATUS_FLAG_THROTTLED | STATUS_FLAG_OVERHEATING |
                                               STATUS_FLAG_LOW_MEMORY | STATUS_FLAG_LOW_DISK;

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_CompanionHealth *_singleton;

    // thresholds for the DEGRADED and CRITICAL states
    static constexpr uint8_t LOAD_WARN_PCT = 80;
    static constexpr uint8_t LOAD_CRITICAL_PCT = 95;
    static constexpr int16_t TEMP_WARN_CDEGC = 7500;      // 75.0C
    static constexpr int16_t TEMP_CRITICAL_CDEGC = 9000;  // 90.0C

    // bounds applied to CCH_TIMEOUT before use, matching the parameter range
    static constexpr float TIMEOUT_MIN_MS = 2000.0f;
    static constexpr float TIMEOUT_MAX_MS = 120000.0f;

    // parameters
    AP_Int8 _fs_enable;      // failsafe action (0=disabled, 1-7 same as GCS)
    AP_Float _fs_timeout;    // timeout in seconds before failsafe
    AP_Int32 _svc_mask;      // bitmask of services that must be running

    // state
    uint32_t _last_msg_ms;          // timestamp of last received message
    uint32_t _last_log_ms;          // timestamp of last DataFlash log
    uint16_t _last_watchdog_seq;    // last received watchdog sequence
    uint32_t _watchdog_last_changed_ms; // timestamp when watchdog sequence last changed
    State _state;                   // current connection/health state
    State _last_reported_state;     // last state announced to the GCS
    CompanionStatus _status;        // latest received status

    // update state based on current metrics
    void update_state();

    // log current health metrics
    void Log_Write_CCH() const;
};

namespace AP {
    AP_CompanionHealth *companion_health();
};

#endif  // AP_COMPANION_HEALTH_ENABLED
