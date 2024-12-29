#ifndef INC_DERIVED_HAL_H_
#define INC_DERIVED_HAL_H_

#include "hal.h"
#include "plane.h"
#include <gnss.h>
#include <icm42688p.h>
#include <ina219.h>
#include <mlx90393.h>
#include <sd.h>

extern "C"
{
#include "barometer.h"
#include "main.h"
}

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart3;

class Derived_hal : public HAL
{
public:
	Derived_hal(Plane * plane);

	void init();
	void read_sensors();

	// imu_hal.cpp
	void init_imu();
	void read_imu();

	// baro_hal.cpp
	void init_baro();
	void read_baro();

	// gnss_hal.cpp
	void init_gnss();
	void read_gnss();
	void gnss_dma_complete();

	// compass_hal.cpp
	void init_compass();
	void read_compass();

	// logger_hal.cpp
	void init_logger();
	void write_sd();
	void flush_sd();
	void read_sd();

	// debug_hal.cpp
	void debug_print(char * str);
	void usb_print(char * str);
	void toggle_led();

	// time_hal.cpp
	void delay_us(uint64_t);
	uint64_t get_time_us();

	// power_monitor_hal.cpp
	void read_power_monitor();

private:
	Plane * _plane;
	ICM42688 _imu;
	INA219 _ina219;
	Adafruit_MLX90393 _mag;
	GNSS _gnss;
	Sd _sd;

	float _hard_iron[3] = {52.67, -5.27, 81.54};
	float _soft_iron[3][3] = {{1.031, 0.015, -0.0032},
	                         {0.015, 0.967, -0.025},
	                         {-0.032, -0.025, 1.005}};
};

#endif /* INC_DERIVED_HAL_H_ */
