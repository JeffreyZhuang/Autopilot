#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "modules/ahrs/ahrs.h"
#include "modules/attitude_control/attitude_control.h"
#include "modules/commander/commander.h"
#include "modules/l1_controller/l1_controller.h"
#include "modules/mixer/mixer.h"
#include "modules/navigator/navigator.h"
#include "modules/position_estimator/position_estimator.h"
#include "modules/rc_handler/rc_handler.h"
#include "modules/storage/storage.h"
#include "modules/tecs/tecs.h"
#include "modules/telemetry/telem.h"
#include "hal.h"
#include <stdint.h>
#include <stdio.h>
#include <cstring>

class Autopilot
{
public:
	Autopilot(HAL* hal);

    void setup();

private:
    HAL* _hal;
    AHRS _ahrs;
    Position_estimator _position_estimator;
    Attitude_control _att_control;
    L1_controller _l1_controller;
    Telem _telem;
    Storage _storage;
    Mixer _mixer;
    Rc_handler _rc_handler;
    Commander _commander;
    Tecs _tecs;
    Navigator _navigator;

    // Helper functions
    void update_time();
    void debug_serial();

    // Scheduler
    void main_task();
	void background_task();
    static void static_main_task() { _instance->main_task(); }
    static void static_background_task() { _instance->background_task(); }

    static Autopilot* _instance;
};

#endif
