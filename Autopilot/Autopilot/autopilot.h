#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <stdint.h>
#include <stdio.h>
#include <cstring>
#include <plane.h>
#include <hal.h>
#include <ahrs.h>
#include <navigation.h>

/**
 * @brief Autopilot
 */
class Autopilot
{
public:
    Autopilot(HAL * hal, Plane * plane);

    void init();

    void main_task();
    void logger_task();
private:
    HAL * _hal;
    Plane * _plane;
    AHRS _ahrs;
    Navigation _navigation;
};

#endif
