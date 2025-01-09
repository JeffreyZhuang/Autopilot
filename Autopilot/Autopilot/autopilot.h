#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <stdint.h>
#include <stdio.h>
#include <cstring>
#include "hal.h"
#include "ahrs.h"
#include "navigation.h"
#include "commander.h"
#include "control.h"
#include "guidance.h"

/**
 * @brief Autopilot
 */
class Autopilot
{
public:
	Autopilot(HAL* hal, Plane* plane);

    void init();

    void main_task();
    static void static_main_task()
    {
    	if (_instance)
    	{
    		_instance->main_task();
    	}
    }

    void logger_task();
    static void static_logger_task()
	{
		if (_instance)
		{
			_instance->logger_task();
		}
	}

    static Autopilot *get_instance() { return _instance; };

private:
    HAL* _hal;
    Plane* _plane;
    AHRS _ahrs;
    Navigation _navigation;
    Commander _commander;
    Control _control;
    Guidance _guidance;

    static Autopilot* _instance;
};

#endif
