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

AP_CompanionHealth *AP_CompanionHealth::_singleton;

const AP_Param::GroupInfo AP_CompanionHealth::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Companion Computer Failsafe Enable
    // @Description: Action to take when companion computer stops sending health messages. Uses same values as GCS failsafe.
    // @Values: 0:Disabled,1:RTL,2:Continue Mission in Auto,3:SmartRTL or RTL,4:SmartRTL or Land,5:Land,6:Auto DO_LAND_START or RTL,7:Brake or Land
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1, AP_CompanionHealth, _fs_enable, 0),

    // @Param: TIMEOUT
    // @DisplayName: Companion Computer Failsafe Timeout
    // @Description: Time in seconds without companion health messages before failsafe triggers
    // @Units: s
    // @Range: 1 60
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TIMEOUT", 2, AP_CompanionHealth, _fs_timeout, 5),

    AP_GROUPEND
};

AP_CompanionHealth::AP_CompanionHealth()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);

    // initialize state
    _last_msg_ms = 0;
    _last_watchdog_seq = 0;
    _healthy = false;
    memset(&_status, 0, sizeof(_status));
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

    _last_watchdog_seq = packet.watchdog_seq;
    _last_msg_ms = AP_HAL::millis();
    _healthy = true;

    // announce first connection
    if (was_never_connected) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Companion computer connected");
    }
}

uint32_t AP_CompanionHealth::last_message_age_ms() const
{
    if (_last_msg_ms == 0) {
        return UINT32_MAX;
    }
    return AP_HAL::millis() - _last_msg_ms;
}

void AP_CompanionHealth::update()
{
    // disabled - nothing to do
    if (_fs_enable <= 0) {
        return;
    }

    // never connected - don't trigger failsafe
    if (_last_msg_ms == 0) {
        return;
    }

    const uint32_t timeout_ms = uint32_t(_fs_timeout * 1000.0f);
    const uint32_t age_ms = last_message_age_ms();

    if (age_ms > timeout_ms) {
        _healthy = false;
    } else {
        _healthy = true;
    }
}

namespace AP {

AP_CompanionHealth *companion_health()
{
    return AP_CompanionHealth::get_singleton();
}

}

#endif  // AP_COMPANION_HEALTH_ENABLED
