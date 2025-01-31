#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <stdint.h>
#include <stdio.h>
#include <cstring>
#include "hal.h"
#include "ahrs.h"
#include "navigation.h"
#include "control.h"
#include "guidance.h"
#include "telem.h"

/**
 * @brief Autopilot
 */
class Autopilot
{
public:
	Autopilot(HAL* hal, Plane* plane);

    void run();

    static Autopilot *get_instance() { return _instance; }

private:
    HAL* _hal;
    Plane* _plane;
    AHRS _ahrs;
    Navigation _navigation;
    Control _control;
    Guidance _guidance;
    Telem _telem;

    // States
    void takeoff();
    void mission();
    void land();

    void evaluate_auto_mode();
    void evaluate_manual_mode();
    void update_time();
    void init_state();

    void main_task();
	void logger_task();
    static void static_main_task() { _instance->main_task(); }
    static void static_logger_task() { _instance->logger_task(); }

    static Autopilot* _instance;
};

#endif
