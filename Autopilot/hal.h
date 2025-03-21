#ifndef HAL_H_
#define HAL_H_

#include "plane.h"

enum class Hal_mode
{
	FLIGHT,
	HITL
};

class HAL
{
public:
	virtual ~HAL() = default;

    virtual void init() = 0;

    // Sensors
    void read_sensors();

    // Telemetry
    virtual void transmit_telem(uint8_t tx_buff[], int len) = 0;
    virtual bool read_telem(uint8_t* byte) = 0;
    virtual bool telem_buffer_empty() = 0;
    virtual void get_rc_input(uint16_t duty[], uint8_t num_channels) = 0;

    // Logger
    virtual void write_storage_buffer(uint8_t* packet, uint16_t len) = 0;
    virtual void flush_storage_buffer() = 0;
    virtual void read_storage(uint8_t* rx_buff, uint16_t size) = 0;

    // Debug
    virtual void debug_print(char* str) = 0;
    virtual void toggle_led() = 0;
    void usb_print(char* str);

    // Control surfaces
    void set_pwm(uint16_t ele_duty, uint16_t rud_duty, uint16_t thr_duty,
				 uint16_t aux1_duty, uint16_t aux2_duty, uint16_t aux3_duty);

    // Time
    virtual void delay_us(uint64_t us) = 0;
    virtual uint64_t get_time_us() const = 0;

    // Scheduler
    virtual void start_main_task(void (*task)()) = 0;
    virtual void start_background_task(void (*task)()) = 0;

    // HITL
    void enable_hitl() noexcept;

private:
    Hal_mode _hal_mode = Hal_mode::FLIGHT;

    virtual void read_sensors_flight() = 0;
    virtual void read_sensors_hitl() = 0;

    virtual void set_pwm_flight(uint16_t ele_duty, uint16_t rud_duty, uint16_t thr_duty,
								uint16_t aux1_duty, uint16_t aux2_duty, uint16_t aux3_duty) = 0;
    virtual void set_pwm_hitl(uint16_t ele_duty, uint16_t rud_duty, uint16_t thr_duty,
    						  uint16_t aux1_duty, uint16_t aux2_duty, uint16_t aux3_duty) = 0;

    virtual void usb_print_flight(char* str) = 0;
};

#endif
