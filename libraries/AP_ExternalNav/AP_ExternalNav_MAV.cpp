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

#include <AP_HAL/AP_HAL.h>
#include "AP_ExternalNav_MAV.h"
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_ExternalNav_MAV::AP_ExternalNav_MAV(AP_ExternalNav &frontend) :
    AP_ExternalNav_Backend(frontend)
{
}

// consume EXT_NAV MAVLink message
void AP_ExternalNav_MAV::handle_msg(mavlink_message_t *msg)
{
    // decode message
    mavlink_ext_nav_t packet;
    mavlink_msg_ext_nav_decode(msg, &packet);

    const Vector3f sensor_offset(packet.offset_x, packet.offset_y, packet.offset_z );
    const Vector3f position_estimate(packet.pos_x, packet.pos_y, packet.pos_z);
    const Quaternion orientation_estimate(packet.quat_q1, packet.quat_q2, packet.quat_q3, packet.quat_q4);
    set_estimate(packet.scale_unknown, packet.frame_is_NED,
                 sensor_offset, position_estimate, orientation_estimate,
                 packet.pos_error, packet.ang_error,
                 packet.time_measurement_msec, packet.time_reset_msec);
}
