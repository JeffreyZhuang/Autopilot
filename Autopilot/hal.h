#ifndef HAL_H
#define HAL_H

#include <plane.h>
#include <stdint.h>
#include <stdio.h>
#include <cstring>

class HAL {
public:
    HAL(Plane * plane);

    virtual void setup() = 0;
    virtual void poll() = 0;
    virtual void toggle_led() = 0;
    virtual void delay_us(uint32_t us) = 0;
    virtual void swo_print(char * str) = 0;
    virtual void usb_print(char * str) = 0;
    virtual void write_sd() = 0;
    virtual uint32_t get_time_us() = 0;
private:
    Plane * _plane;
};

#endif

// Hardware abstraction layer
// All hardware dependent code goes here

// Navigation polls from sensors. Poll IMU data from AHRS.
// AP_NavEKF2
// Look at NavEKF2. DAL? Data abstraction layer? So, both AHRS and navigation read from sensor. Use DAL to handle FIFO buffer (I guess this is better because you can use one independent of the other)
// GCS: https://github.com/blauret/pyG5