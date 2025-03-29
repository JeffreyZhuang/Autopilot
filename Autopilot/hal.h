#ifndef HAL_H_
#define HAL_H_

#include <data_bus.h>

class HAL
{
public:
	virtual ~HAL() = default;

    virtual void init() = 0;

    // Sensors
    virtual bool read_imu(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) = 0;
    virtual bool read_mag(float *mx, float *my, float *mz) = 0;
    virtual bool read_baro(float *alt) = 0;
    virtual bool read_gnss(double *lat, double *lon, float* alt, uint8_t* sats, bool* fix) = 0;
    virtual bool read_optical_flow(int16_t *x, int16_t *y) = 0;
    virtual bool read_power_monitor(float *voltage, float* current) = 0;

    // Telemetry
    virtual void transmit_telem(uint8_t tx_buff[], int len) = 0;
    virtual bool read_telem(uint8_t* byte) = 0;
    virtual bool telem_buffer_empty() = 0;
    virtual void get_rc_input(uint16_t duty[], uint8_t num_channels) = 0;

    // Logger
    virtual void write_storage_buffer(uint8_t* packet, uint16_t len) = 0;
    virtual void flush_storage_buffer() = 0;
    virtual bool read_storage(uint8_t* rx_buff, uint16_t size) = 0;

    // Debug
    virtual void debug_print(char* str) = 0;
    virtual void toggle_led() = 0;

    // USB
    virtual bool usb_buffer_empty() = 0;
    virtual void usb_transmit(uint8_t buf[], int len) = 0;
    virtual bool usb_read(uint8_t *byte) = 0;

    // Control surfaces
    virtual void set_pwm(uint16_t ele_duty, uint16_t rud_duty, uint16_t thr_duty,
				 	     uint16_t aux1_duty, uint16_t aux2_duty, uint16_t aux3_duty) = 0;

    // Time
    virtual void delay_us(uint64_t us) = 0;
    virtual uint64_t get_time_us() const = 0;

    // Scheduler
    virtual void start_main_task(void (*task)(void*), void* arg) = 0;
    virtual void start_background_task(void (*task)(void*), void* arg) = 0;
};

#endif
