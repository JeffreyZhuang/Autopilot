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
	void setup();
	void poll();
	void write_sd();
	void swo_print(char * str);
	void usb_print(char * str);
	void i2c_scan();
	void toggle_led();
	void delay_us(uint64_t);
	uint64_t get_time_us();
	void gnss_callback();
private:
	Plane * _plane;
	ICM42688 imu;
	INA219 ina219;
	Adafruit_MLX90393 mag;
	GNSS gnss;
	Sd sd;
};

#endif /* INC_DERIVED_HAL_H_ */
