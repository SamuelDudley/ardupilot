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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_ExternalNav.h"

class AP_ExternalNav_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_ExternalNav_Backend(AP_ExternalNav &frontend);

    // consume EXT_NAV MAVLink message
	virtual void handle_msg(mavlink_message_t *msg) {};

protected:

    // set estimate (used by backend to update state)
    void set_estimate(bool scale_unknown, bool frame_is_NED,
            const Vector3f &sensor_offset, const Vector3f &position_estimate, const Quaternion &orientation_estimate,
            float position_error, float orientation_error,
            uint32_t source_timestamp_ms, uint32_t last_reset_ms);

private:

    // references
    AP_ExternalNav &_frontend;
};
