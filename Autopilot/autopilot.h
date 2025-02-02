#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <stdint.h>
#include <stdio.h>
#include <cstring>
#include "hal.h"
#include "Modules/AHRS/ahrs.h"
#include "Modules/Navigation/navigation.h"
#include "Modules/Control/control.h"
#include "Modules/Guidance/guidance.h"
#include "Modules/Telemetry/telem.h"

/**
 * @brief Autopilot
 */
class Autopilot
{
public:
	Autopilot(HAL* hal, Plane* plane);

    void setup();

    static Autopilot *get_instance() { return _instance; }

private:
    HAL* _hal;
    Plane* _plane;
    AHRS _ahrs;
    Navigation _navigation;
    Control _control;
    Guidance _guidance;
    Telem _telem;

    // System mode
    void evaluate_system_mode();
	void boot();
	void flight();

    // Auto mode
    void evaluate_auto_mode();
    void takeoff();
    void mission();
    void land();

	// Manual mode
    void evaluate_manual_mode();

    // Helper functions
    void update_time();
    void init_state();
    void set_ahrs_initial();

    // Scheduler
    void main_task();
	void logger_task();
    static void static_main_task() { _instance->main_task(); }
    static void static_logger_task() { _instance->logger_task(); }

    static Autopilot* _instance;
};

#endif
