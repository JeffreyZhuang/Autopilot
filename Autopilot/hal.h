#ifndef HAL_H_
#define HAL_H_

#include "plane.h"

/**
 * @brief Hardware abstraction layer
 */
class HAL
{
public:
    virtual void init() = 0;

    // Sensors
    virtual void read_sensors() = 0;

    // Telemetry
    virtual void transmit_telem(uint8_t tx_buff[], int len) = 0;
    virtual bool read_telem() = 0;

    // Logger
    virtual void write_storage_buffer(uint8_t* packet, uint16_t len) = 0;
    virtual void flush_storage_buffer() = 0;
    virtual void read_storage(uint8_t* rx_buff, uint16_t size) = 0;

    // Debug
    virtual void debug_print(char* str) = 0;
    virtual void usb_print(char* str) = 0;
    virtual void toggle_led() = 0;

    // Control surfaces
    virtual void set_elevator_duty(uint16_t duty_us) = 0;
    virtual void set_rudder_duty(uint16_t duty_us) = 0;
    virtual void set_throttle_duty(uint16_t duty_us) = 0;

    // Time
    virtual void delay_us(uint64_t us) = 0;
    virtual uint64_t get_time_us() = 0;

    // Scheduler
    virtual void set_main_task(void (*task)()) = 0;
    virtual void set_background_task(void (*task)()) = 0;
    virtual float get_main_dt() const = 0;

    virtual ~HAL() {}
};

#endif
