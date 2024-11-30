#include <stdint.h>
#include <stdio.h>
#include <cstring>
#include <plane.h>
#include <hal.h>
#include <ahrs.h>
#include <navigation.h>

class Autopilot {
public:
    Autopilot();
    void setup();
    void loop();
private:
    Plane plane;
    AHRS ahrs;
    HAL hal;
    Navigation navigation;

    uint32_t prev_loop_time;
    uint32_t prev_print_time;
    char txBuf[500];
};