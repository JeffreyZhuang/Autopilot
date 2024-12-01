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
    virtual void i2c_scan() = 0;
    virtual uint32_t get_time_us() = 0;
private:
    Plane * _plane;
};

#endif

// Hardware abstraction layer


// GCS: https://github.com/blauret/pyG5