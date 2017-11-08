/*
 *  Camera wrapper class for optional tight camera autopilot synchronization
 *  Camera Vision overloads the camera functions to add behaviors suited to machine vision
 *
 *  Samuel Dudley 28/10/2017
 *
 *  dudley.samuel@gmail.com
 *
 */

#pragma once

#include "AP_Camera.h"
#include <AP_AHRS/AP_AHRS.h>

#define AP_CAMERA_VISION_DEFAULT_FEEDBACK_COMPONENT_ID   191
#define AP_CAMERA_VISION_DEFAULT_GCS_FEEDBACK_HZ         1 // 1 message per second

class AP_Camera_Vision: public AP_Camera {
public:
    static AP_Camera_Vision create(AP_Relay *obj_relay,
                            uint32_t _log_camera_bit,
                            const struct Location &_loc,
                            const AP_GPS &_gps,
                            AP_AHRS &_ahrs) {
        return AP_Camera_Vision{obj_relay, _log_camera_bit, _loc, _gps, _ahrs};
    }

    constexpr AP_Camera_Vision(AP_Camera_Vision &&other) = default;

    /* Do not allow copies */
    AP_Camera_Vision(const AP_Camera_Vision &other) = delete;
    AP_Camera_Vision &operator=(const AP_Camera_Vision&) = delete;

    // overloaded function from the standard AP_Camera class
    // check to see if a trigger event has occurred and action accordingly
    void update_trigger();

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Camera_Vision(AP_Relay *obj_relay, uint32_t _log_camera_bit,
            const struct Location &_loc, const AP_GPS &_gps, const AP_AHRS &_ahrs)
            : AP_Camera(obj_relay, _log_camera_bit, _loc, _gps, _ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // determine if the GCS should be informed about this image capture event
    bool should_send_feedback_to_gcs(void);

    // component ID of the CC which will receive the AHRS summary MAVLink message
    AP_Int16 _vision_feedback_target_component;

    uint32_t _last_gcs_feedback_time;

    // the cameras local copy of the AHRS summary structure
    AP_AHRS::AHRS_Summary _ahrs_summary;

    // maximum rate at which the GCS will receive camera feedback information
    AP_Float _gcs_feedback_hz;
};
