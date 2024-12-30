#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <stdint.h>
#include <stdio.h>
#include <cstring>
#include <hal.h>
#include <ahrs.h>
#include <navigation.h>
#include "commander.h"
#include "control.h"

/**
 * @brief Autopilot
 */
class Autopilot
{
public:
    Autopilot(HAL* hal, Plane* plane);

    void init();

    void main_task();
    void logger_task();
private:
    HAL* _hal;
    Plane* _plane;
    AHRS _ahrs;
    Navigation _navigation;
    Commander _commander;
    Control _control;
};

#endif
