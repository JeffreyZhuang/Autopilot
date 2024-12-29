#ifndef HAL_H_
#define HAL_H_

#include <plane.h>
#include <stdint.h>
#include <stdio.h>
#include <cstring>

/**
 * @brief Hardware abstraction layer
 *
 */
class HAL
{
public:
    HAL(Plane * plane);

    virtual void setup() = 0;

    // Sensors
    virtual void poll() = 0;

    // Datalogging
    virtual void write_sd() = 0;
    virtual void read_sd() = 0;

    // Debug
    virtual void swo_print(char * str) = 0;
    virtual void usb_print(char * str) = 0;
    virtual void i2c_scan() = 0;
    virtual void toggle_led() = 0;

    // Time
    virtual void delay_us(uint64_t us) = 0;
    virtual uint64_t get_time_us() = 0;
private:
    Plane * _plane;
};

#endif
