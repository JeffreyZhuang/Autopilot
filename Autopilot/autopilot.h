#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <modules/attitude_control/attitude_control.h>
#include <modules/position_control/position_control.h>
#include <modules/position_estimator/position_estimator.h>
#include "modules/ahrs/ahrs.h"
#include "modules/commander/commander.h"
#include "modules/mixer/mixer.h"
#include "modules/navigator/navigator.h"
#include "modules/rc_handler/rc_handler.h"
#include "modules/storage/storage.h"
#include "modules/telemetry/telem.h"
#include "modules/sensors/sensors.h"
#include "modules/usb_comm/usb_comm.h"
#include "hal.h"
#include <stdint.h>
#include <stdio.h>
#include <cstring>

class Autopilot
{
public:
	Autopilot(HAL* hal, Data_bus* data_bus);

    void setup();

private:
    HAL* _hal;
    AHRS _ahrs;
    PositionEstimator _position_estimator;
    AttitudeControl _att_control;
    PositionControl _position_control;
    Telem _telem;
    Storage _storage;
    Mixer _mixer;
    Rc_handler _rc_handler;
    Commander _commander;
    Navigator _navigator;
    Sensors _sensors;
    USBComm _usb_comm;

    // Scheduler
    void main_task();
    static void static_main_task() { _instance->main_task(); }

    static Autopilot* _instance;
};

#endif
