#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <Modules/Mixer/mixer.h>
#include <Modules/RC/rc_handler.h>
#include "Modules/AHRS/ahrs.h"
#include "Modules/Navigation/navigation.h"
#include "Modules/Control/control.h"
#include "Modules/Guidance/guidance.h"
#include "Modules/Telemetry/telem.h"
#include "Modules/Storage/storage.h"
#include "hal.h"
#include <stdint.h>
#include <stdio.h>
#include <cstring>

/**
 * @brief Autopilot
 */
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
    Control_allocator _control_allocator;
    Rc_handler _rc_handler;

    // System mode
    void evaluate_system_mode();
	void boot();
	void flight();

    // Auto mode
    void evaluate_auto_mode();
    void ready();
    void takeoff();
    void mission();
    void land();
    void flare();
    void touchdown();

	// Manual mode
    void evaluate_manual_mode();

    // Helper functions
    void update_time();
    void init_state();
    void debug_serial();

    // Scheduler
    void main_task();
	void logger_task();
    static void static_main_task() { _instance->main_task(); }
    static void static_logger_task() { _instance->logger_task(); }

    static Autopilot* _instance;
};

#endif
