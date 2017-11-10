#pragma once

#include "AP_ExternalNav_Backend.h"

class AP_ExternalNav_MAV : public AP_ExternalNav_Backend
{

public:
    // constructor
    AP_ExternalNav_MAV(AP_ExternalNav &frontend);

    // consume VISION_POSITION_DELTA MAVLink message
    void handle_msg(mavlink_message_t *msg) override;
};
