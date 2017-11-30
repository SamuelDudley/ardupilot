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

#include "AP_ExternalNav.h"
#include "AP_ExternalNav_Backend.h"
#include "AP_ExternalNav_MAV.h"

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_ExternalNav::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: External navigation sensor connection type
    // @Description: External navigation sensor connection type
    // @Values: 0:None,1:MAV
    // @User: Advanced
    AP_GROUPINFO("_TYPE", 0, AP_ExternalNav, _type, 0),

    AP_GROUPEND
};

AP_ExternalNav::AP_ExternalNav()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// detect and initialise any sensors
void AP_ExternalNav::init()
{
    // create backend
    if (_type == AP_ExternalNav_Type_MAV) {
        _driver = new AP_ExternalNav_MAV(*this);
    }
}

// return true if sensor is enabled
bool AP_ExternalNav::enabled() const
{
    return ((_type != AP_ExternalNav_Type_None) && (_driver != nullptr));
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_ExternalNav::healthy() const
{
    if (!enabled()) {
        return false;
    }

    // healthy if we have received sensor messages within the past 300ms
    return ((AP_HAL::millis() - _state.last_update_ms) < AP_EXTERNALNAV_TIMEOUT_MS);
}

// consume EXT_NAV MAVLink message
void AP_ExternalNav::handle_msg(mavlink_message_t *msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->handle_msg(msg);
    }
}

