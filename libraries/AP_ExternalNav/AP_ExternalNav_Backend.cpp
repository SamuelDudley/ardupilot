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

#include "AP_ExternalNav_Backend.h"

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_ExternalNav_Backend::AP_ExternalNav_Backend(AP_ExternalNav &frontend) :
    _frontend(frontend)
{
}

// set deltas (used by backend to update state)
void AP_ExternalNav_Backend::set_estimate(bool scale_unknown, bool frame_is_NED,
            const Vector3f &sensor_offset, const Vector3f &position_estimate, const Quaternion &orientation_estimate,
            float position_error, float orientation_error,
            uint32_t source_timestamp_ms, uint32_t last_reset_ms)
{
    _frontend._state.scale_unknown = scale_unknown;
    _frontend._state.frame_is_NED = frame_is_NED;
    _frontend._state.sensor_offset = sensor_offset;
    _frontend._state.position_estimate = position_estimate;
    _frontend._state.orientation_estimate = orientation_estimate;
    _frontend._state.position_error = position_error;
    _frontend._state.orientation_error = orientation_error;
    _frontend._state.source_timestamp_ms = source_timestamp_ms;
    _frontend._state.last_reset_ms = last_reset_ms;

    _frontend._state.last_update_ms = AP_HAL::millis();
}
