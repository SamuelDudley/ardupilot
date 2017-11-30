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
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_ExternalNav_Backend;

#define AP_EXTERNALNAV_TIMEOUT_MS 300

class AP_ExternalNav
{
public:
    friend class AP_ExternalNav_Backend;

    AP_ExternalNav();

    // external position backend types (used by _TYPE parameter)
    enum AP_ExternalNav_Type {
        AP_ExternalNav_Type_None   = 0,
        AP_ExternalNav_Type_MAV    = 1
    };

    // The ExternalNavState structure is filled in by the backend driver
    struct ExternalNavState {
        bool scale_unknown;
        bool frame_is_NED;
        Vector3f sensor_offset;
        Vector3f position_estimate;
        Quaternion orientation_estimate;
        float position_error;
        float orientation_error;
        uint32_t source_timestamp_ms;
        uint32_t last_reset_ms;
        uint32_t last_update_ms;    // system time (in milliseconds) of last update from sensor
    };

    // detect and initialise any sensors
    void init();

    // return true if sensor is enabled
    bool enabled() const;

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() const;

    // state accessors
    bool get_scale_flag() const { return _state.scale_unknown; }
    bool get_ned_flag() const { return _state.frame_is_NED; }
    // return a 3D vector defining the position offset of the camera in meters relative to the body frame origin
    const Vector3f &get_sensor_offset() const { return _state.sensor_offset; }
    const Vector3f &get_position_estimate() const { return _state.position_estimate; }
    const Quaternion &get_orientation_estimate() const { return _state.orientation_estimate; }
    float get_position_error() const { return _state.position_error; }
    float get_orientation_error() const { return _state.orientation_error; }
    uint32_t get_source_timestamp_ms() const { return _state.source_timestamp_ms; }
    uint32_t get_last_reset_ms() const { return _state.last_reset_ms; }
    uint32_t get_last_update_ms() const { return _state.last_update_ms; }

    // consume VISUAL_POSITION_DELTA data from MAVLink messages
    void handle_msg(mavlink_message_t *msg);

    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Int8 _type;

    // reference to backends
    AP_ExternalNav_Backend *_driver;

    // state of backend
    ExternalNavState _state;
};
