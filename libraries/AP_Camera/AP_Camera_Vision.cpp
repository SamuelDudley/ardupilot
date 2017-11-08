/*
 *  Camera wrapper class for optional tight camera autopilot synchronization
 *  Camera Vision overloads the camera functions to add behaviors suited to machine vision
 *
 *  Samuel Dudley 28/10/2017
 *
 *  dudley.samuel@gmail.com
 *
 */
#include "AP_Camera_Vision.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_Camera_Vision::var_info[] = {
    // parameters from parent class
    AP_NESTEDGROUPINFO(AP_Camera, 0),

    // @Param: FEEDBACK_ID
    // @DisplayName: Camera feedback target component ID
    // @Description: Target ID of the component which handles camera feedback AHRS MAVLink messages
    // @User: Standard
    AP_GROUPINFO("FEEDBACK_ID",  1, AP_Camera_Vision, _vision_feedback_target_component, AP_CAMERA_VISION_DEFAULT_FEEDBACK_COMPONENT_ID),

    // @Param: MAX_GCS_RATE
    // @DisplayName: Max camera feedback message Hz
    // @Description: Maximum rate that camera feedback messages will be sent to the GCS (Hz). 0 = suppress all feedback messages to the GCS, -1 = send all feedback messages to the GCS
    // @Units: Hz
    // @Range: -1 100
    // @User: Advanced
    AP_GROUPINFO("MAX_GCS_RATE", 2, AP_Camera_Vision, _gcs_feedback_hz, AP_CAMERA_VISION_DEFAULT_GCS_FEEDBACK_HZ),

    AP_GROUPEND
};

// check the camera trigger status and action
// this function is called by Arduplane.cpp at 50Hz by default but needs to be called
// at 2 x frame rate of camera to ensure all frames are reported
void AP_Camera_Vision::update_trigger() {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    snapshot_ahrs();
#endif
    // if the hardware trigger has not been installed then do it now
    setup_feedback_callback();
    if (check_trigger_pin()) {
        // registered that a photo was taken (via the feedback pin)
        _image_index++;

        // send a low rate message feedback to the GCS while sending full rate to the CC
        if (should_send_feedback_to_gcs()) {
            gcs().send_message(MSG_CAMERA_FEEDBACK); // this calls send_feedback in the standard camera library
            _last_gcs_feedback_time = AP_HAL::millis();
        }
        // send feedback info and AHRS state to the CC
        send_feedback_ahrs();

        // store the image capture info to dataflash
        DataFlash_Class *df = DataFlash_Class::instance();
        if (df != nullptr) {
            if (df->should_log(log_camera_bit)) {
                // log the AHRS synchronised camera data
                df->Log_Write_Camera_Vision1(_ahrs_summary, _camera_feedback_time, _image_index);
                df->Log_Write_Camera_Vision2(_ahrs_summary, _camera_feedback_time, _image_index);
                // log the default camera data
                df->Log_Write_Camera(ahrs, gps, current_loc);
            }
        }
    }
}



bool AP_Camera_Vision::should_send_feedback_to_gcs(void) {
    if (is_negative(_gcs_feedback_hz)) {
        return true;
    } else if (is_zero(_gcs_feedback_hz)) {
        // handle the zero case here to avoid the divide by zero in the next case
        return false;
    } else if (AP_HAL::millis() - _last_gcs_feedback_time > (1.0/_gcs_feedback_hz)*1000) {
        // sufficient time has passed since we last sent a msg to the GCS
       return true;
    } else {
        // insufficient time has passed, don't send a message to the GCS this image
       return false;
    }
}
