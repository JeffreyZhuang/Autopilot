#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "Modules/Mixer/mixer.h"
#include "Modules/RC/rc_handler.h"
#include "Modules/AHRS/ahrs.h"
#include "Modules/Navigation/navigation.h"
#include "Modules/Control/control.h"
#include "Modules/Guidance/guidance.h"
#include "Modules/Telemetry/telem.h"
#include "Modules/Storage/storage.h"
#include "Modules/Commander/commander.h"
#include "Modules/TECS/tecs.h"
#include "hal.h"
#include <stdint.h>
#include <stdio.h>
#include <cstring>

class Autopilot
{
public:
	Autopilot(HAL* hal, Plane* plane);
    void setup();
private:
    HAL* _hal;
    Plane* _plane;
    AHRS _ahrs;
    Navigation _navigation;
    Control _control;
    Guidance _guidance;
    Telem _telem;
    Storage _storage;
    Mixer _mixer;
    Rc_handler _rc_handler;
    Commander _commander;
    Tecs _tecs;

    // Helper functions
    void update_time();
    void init_state();
    void debug_serial();

    // Scheduler
    void main_task();
	void background_task();
    static void static_main_task() { _instance->main_task(); }
    static void static_background_task() { _instance->background_task(); }

    static Autopilot* _instance;
};

#endif
