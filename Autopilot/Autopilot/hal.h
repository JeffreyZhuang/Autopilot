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

    virtual void init() = 0;

    // Sensors
    virtual void read_sensors() = 0;

    // Logger
    virtual void write_storage_buffer() = 0;
    virtual void flush_storage_buffer() = 0;
    virtual void read_storage() = 0;

    // Debug
    virtual void debug_print(char * str) = 0;
    virtual void usb_print(char * str) = 0;
    virtual void toggle_led() = 0;

    // Servos
    virtual void set_elevator(float deg) = 0;
    virtual void set_rudder(float deg) = 0;

    // Time
    virtual void delay_us(uint64_t us) = 0;
    virtual uint64_t get_time_us() = 0;
private:
    Plane * _plane;
};

#endif
