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

#include "AP_CompanionHealth.h"

#if AP_COMPANION_HEALTH_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include "LogStructure.h"

AP_CompanionHealth *AP_CompanionHealth::_singleton;

const AP_Param::GroupInfo AP_CompanionHealth::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Companion Computer Failsafe Enable
    // @Description: Action to take when the companion computer is unhealthy: no message for CCH_TIMEOUT seconds, a frozen watchdog sequence, a critical resource report, or a required service stopped. Uses the same action values as FS_GCS_ENABLE.
    // @Values: 0:Disabled/NoAction,1:RTL,3:SmartRTL or RTL,4:SmartRTL or Land,5:Land,6:Auto DO_LAND_START/DO_RETURN_PATH_START or RTL,7:Brake or Land
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, AP_CompanionHealth, _fs_enable, 0),

    // @Param: TIMEOUT
    // @DisplayName: Companion Computer Failsafe Timeout
    // @Description: Time in seconds without companion health messages before failsafe triggers
    // @Units: s
    // @Range: 2 120
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TIMEOUT", 2, AP_CompanionHealth, _fs_timeout, 5),

    // @Param: SVC_MASK
    // @DisplayName: Companion Computer Failsafe Service Mask
    // @Description: Bitmask of companion services that must be running. Bit N corresponds to the Nth entry of the service list configured on the companion computer. If a required service stops being reported the failsafe triggers. 0 disables the check.
    // @Bitmask: 0:Service0,1:Service1,2:Service2,3:Service3,4:Service4,5:Service5,6:Service6,7:Service7,8:Service8,9:Service9,10:Service10,11:Service11,12:Service12,13:Service13,14:Service14,15:Service15,16:Service16,17:Service17,18:Service18,19:Service19,20:Service20,21:Service21,22:Service22,23:Service23,24:Service24,25:Service25,26:Service26,27:Service27,28:Service28,29:Service29,30:Service30,31:Service31
    // @User: Standard
    AP_GROUPINFO("SVC_MASK", 3, AP_CompanionHealth, _svc_mask, 0),

    AP_GROUPEND
};

AP_CompanionHealth::AP_CompanionHealth()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_CompanionHealth must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);

    // initialize state
    _last_msg_ms = 0;
    _last_log_ms = 0;
    _last_watchdog_seq = 0;
    _watchdog_last_changed_ms = 0;
    _state = State::DISCONNECTED;
    _last_reported_state = State::DISCONNECTED;
    memset(&_status, 0, sizeof(_status));
    _status.temperature = INT16_MAX;    // no reading yet
}

const char* AP_CompanionHealth::get_state_name() const
{
    switch (_state) {
        case State::DISCONNECTED: return "DISCONNECTED";
        case State::HEALTHY:      return "HEALTHY";
        case State::DEGRADED:     return "DEGRADED";
        case State::CRITICAL:     return "CRITICAL";
    }
    return "UNKNOWN";
}

void AP_CompanionHealth::handle_message(const mavlink_message_t &msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_COMPANION_HEALTH) {
        return;
    }

    mavlink_companion_health_t packet;
    mavlink_msg_companion_health_decode(&msg, &packet);

    // check if this is first connection
    const bool was_never_connected = (_last_msg_ms == 0);

    // store status
    _status.services_status = packet.services_status;
    _status.watchdog_seq = packet.watchdog_seq;
    _status.temperature = packet.temperature;
    _status.cpu_load = packet.cpu_load;
    _status.memory_used = packet.memory_used;
    _status.disk_used = packet.disk_used;
    _status.gpu_load = packet.gpu_load;
    _status.status_flags = packet.status_flags;

    if (was_never_connected || _last_watchdog_seq != packet.watchdog_seq) {
        _watchdog_last_changed_ms = AP_HAL::millis();
    }
    _last_watchdog_seq = packet.watchdog_seq;
    _last_msg_ms = AP_HAL::millis();

    // announce first connection
    if (was_never_connected) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Companion computer connected");
    }

    // update state based on metrics
    update_state();
}

uint32_t AP_CompanionHealth::last_message_age_ms() const
{
    if (_last_msg_ms == 0) {
        return UINT32_MAX;
    }
    return AP_HAL::millis() - _last_msg_ms;
}

void AP_CompanionHealth::update_state()
{
    // check for critical conditions
    const bool is_overheating = (_status.status_flags & STATUS_FLAG_OVERHEATING) != 0;
    const bool any_flag_set = (_status.status_flags & STATUS_FLAG_ALL) != 0;
    const bool service_failed = (~_status.services_status & uint32_t(_svc_mask.get())) != 0;
    const bool temp_valid = (_status.temperature != INT16_MAX);

    if (is_overheating || _status.cpu_load > LOAD_CRITICAL_PCT || _status.memory_used > LOAD_CRITICAL_PCT ||
        (temp_valid && _status.temperature > TEMP_CRITICAL_CDEGC) || service_failed) {
        _state = State::CRITICAL;
    } else if (any_flag_set || _status.cpu_load > LOAD_WARN_PCT || _status.memory_used > LOAD_WARN_PCT ||
               (temp_valid && _status.temperature > TEMP_WARN_CDEGC)) {
        _state = State::DEGRADED;
    } else {
        _state = State::HEALTHY;
    }
}

void AP_CompanionHealth::update()
{
    // never connected - nothing to do
    if (_last_msg_ms == 0) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    // a zero or negative parameter would make every pass look like a timeout
    const uint32_t timeout_ms = uint32_t(constrain_float(_fs_timeout * 1000.0f, TIMEOUT_MIN_MS, TIMEOUT_MAX_MS));
    const uint32_t age_ms = last_message_age_ms();

    // check for timeout
    if (age_ms > timeout_ms) {
        _state = State::DISCONNECTED;
    } else if (now_ms - _watchdog_last_changed_ms > timeout_ms) {
        _state = State::CRITICAL; // frozen thread
    }

    // report state changes to the GCS
    if (_state != _last_reported_state) {
        _last_reported_state = _state;
        const MAV_SEVERITY severity = (_state == State::HEALTHY) ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING;
        if (_status.temperature == INT16_MAX) {
            GCS_SEND_TEXT(severity, "Companion [%s]: CPU %u%% Mem %u%%",
                          get_state_name(), _status.cpu_load, _status.memory_used);
        } else {
            GCS_SEND_TEXT(severity, "Companion [%s]: CPU %u%% Mem %u%% Temp %.1fC",
                          get_state_name(), _status.cpu_load, _status.memory_used, _status.temperature * 0.01f);
        }
    }

    // log health metrics at 1Hz
    if (now_ms - _last_log_ms >= 1000) {
        _last_log_ms = now_ms;
        Log_Write_CCH();
    }
}

void AP_CompanionHealth::Log_Write_CCH() const
{
#if HAL_LOGGING_ENABLED
    const struct log_CCH pkt{
        LOG_PACKET_HEADER_INIT(LOG_CCH_MSG),
        time_us         : AP_HAL::micros64(),
        state           : (uint8_t)_state,
        services_status : _status.services_status,
        watchdog_seq    : _status.watchdog_seq,
        temperature     : _status.temperature,
        cpu_load        : _status.cpu_load,
        memory_used     : _status.memory_used,
        disk_used       : _status.disk_used,
        gpu_load        : _status.gpu_load,
        status_flags    : _status.status_flags
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
#endif
}

namespace AP {

AP_CompanionHealth *companion_health()
{
    return AP_CompanionHealth::get_singleton();
}

}

#endif  // AP_COMPANION_HEALTH_ENABLED
