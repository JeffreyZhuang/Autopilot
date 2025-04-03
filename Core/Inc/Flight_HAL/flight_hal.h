#ifndef INC_FLIGHT_HAL_H_
#define INC_FLIGHT_HAL_H_

#include <data_bus.h>
#include <Drivers/sbus_input.h>
#include <lib/utils/utils.h>
#include "Drivers/servo.h"
#include "Drivers/gnss.h"
#include "Drivers/icm42688p.h"
#include "Drivers/ina219.h"
#include "Drivers/mlx90393.h"
#include "Drivers/sd.h"
#include "Drivers/uart_stream.h"
#include "Drivers/usb_stream.h"
#include "Drivers/cxof.h"
#include "hal.h"

extern "C"
{
#include "usbd_cdc_if.h"
#include "Drivers/barometer.h"
}

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

class Flight_hal : public HAL
{
public:
	Flight_hal(Data_bus* data_bus);

	void init() override;

	bool read_imu(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) override;
	bool read_mag(float *mx, float *my, float *mz) override;
	bool read_baro(float *alt) override;
	bool read_gnss(double *lat, double *lon, float* alt, uint8_t* sats, bool* fix) override;
	bool read_optical_flow(int16_t *x, int16_t *y) override;
	bool read_power_monitor(float *voltage, float* current) override;

	void init_imu();
	void init_baro();
	void init_gnss();
	void init_compass();
	void init_of();

	static void gnss_dma_complete() { _instance->_gnss.dma_complete(); }
	static void of_dma_complete() { _instance->cxof.dma_complete(); }

	// logger_hal.cpp
	void create_file(char name[], uint8_t len) override;
	bool write_storage(uint8_t byte) override;
	bool read_storage(uint8_t* rx_buff, uint16_t size) override;
	static void sd_interrupt_callback() { _instance->_sd.interrupt_callback(); }

	// debug_hal.cpp
	void debug_print(char * str) override;
	void toggle_led() override;

	// time_hal.cpp
	void delay_us(uint64_t) override;
	uint64_t get_time_us() const override;

	// servos_hal.cpp
	void init_servos();
	void set_pwm(uint16_t ele_duty, uint16_t rud_duty, uint16_t thr_duty,
				 uint16_t aux1_duty, uint16_t aux2_duty, uint16_t aux3_duty) override;

	// power_monitor_hal.cpp
	void read_power_monitor();

	// telemetry_hal.cpp
	void init_telem();
	void transmit_telem(uint8_t tx_buff[], int len) override;
	bool read_telem(uint8_t* byte) override;
	bool telem_buffer_empty() override;
	static void telemetry_dma_complete() { _instance->telem.dma_complete(); }

	// RC
	void get_rc_input(uint16_t duty[], uint8_t num_channels) override;
	static void rc_dma_complete() { _instance->sbus_input.dma_complete(); }

	// USB
	bool usb_buffer_empty() override;
  	void usb_transmit(uint8_t buf[], int len);
	bool usb_read(uint8_t *byte);
	static void usb_rx_callback(uint8_t* Buf, uint32_t Len) { _instance->usb_stream.rx_callback(Buf, Len); };

	// scheduler_hal.cpp
	void set_main_task(void (*task)(void*), void* arg) override;
	void execute_main_task();
	static void main_task_callback() { _instance->execute_main_task(); }

private:
	Data_bus* _data_bus;
	ICM42688 _imu;
	INA219 _ina219;
	Adafruit_MLX90393 _mag;
	GNSS _gnss;
	Sd _sd;
	SBUS_input sbus_input;
	Uart_stream telem;
	Servo servo1;
	Servo servo2;
	Cxof cxof;
	USB_stream usb_stream;

	// scheduler_hal.cpp
	void (*main_task)(void*) = nullptr;
	void (*background_task)(void*) = nullptr;
	void* main_task_arg = nullptr;

	static Flight_hal* _instance;
};

#endif /* INC_FLIGHT_HAL_H_ */
