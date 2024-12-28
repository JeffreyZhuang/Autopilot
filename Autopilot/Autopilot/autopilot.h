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

    void setup();
    void loop();
private:
    HAL * _hal;
    Plane * _plane;

    AHRS ahrs;
    Navigation navigation;

    uint32_t prev_loop_time;
    uint32_t prev_print_time;
    char txBuf[500];
};

#endif
