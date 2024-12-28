/*
 * autopilot_main.cpp
 *
 *  Created on: Dec. 6, 2024
 *      Author: jeffr
 */

#include <autopilot_main.h>
#include <Datalogging.h>
#include <icm42688p.h>
#include <ina219.h>
#include <mlx90393.h>
#include <gnss.h>
#include <plane.h>

extern "C"
{
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include "barometer.h"
#include "main.h"
}

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;

ICM42688 imu(&hspi1, GPIOC, GPIO_PIN_15, SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_4);
INA219 ina219(&hi2c1, 0.01);
Adafruit_MLX90393 mag;
GNSS gnss(&huart3);
Datalogging datalogging;

// Triggered by timer interrupt at 400Hz
uint32_t prev_time;
uint32_t dt;
void main_loop()
{
	uint32_t time = HAL_GetTick();
	dt = time - prev_time;
	prev_time = time;

	float alt = Barometer_getAltitude(true);
	float voltage = ina219.read_voltage();
	float current = ina219.read_current();
	imu.getAGT();
	mag.readDataNonBlocking();

	// Packet to append to micro-SD buffer
	Datalogging_packet p;
	p.time = time;
	p.acc_z = imu.accZ();
	p.alt = alt;
	datalogging.append_buffer(p);

	uint8_t sentence[100];
	if (gnss.parse(sentence))
	{
		char txBuf[200];
		sprintf(txBuf, "lat: %f lon: %f sats: %d %s", gnss.lat, gnss.lon, gnss.sats, sentence);
		CDC_Transmit_FS((uint8_t *)txBuf, strlen(txBuf));
	}

	// USB
//	 char txBuf[200];
//	sprintf(txBuf,
//			"%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
//			time,
//			imu.gyrX(),
//			imu.gyrY(),
//			imu.gyrZ(),
//			imu.accX(),
//			imu.accY(),
//			imu.accZ(),
//			mag.x,
//			mag.y,
//			mag.z,
//			alt,
//			voltage,
//			current);
//	CDC_Transmit_FS((uint8_t *)txBuf, strlen(txBuf));
}

void autopilot_main()
{
	printf("Start\n");

	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5); // LED

	printf("%d\n", imu.begin());

	Barometer_init();
	Barometer_setOSR(OSR_4096);

	mag.begin_SPI(GPIOC, GPIO_PIN_13, &hspi1, SPI_BAUDRATEPRESCALER_8);

	datalogging.initialize();

	gnss.setup();

	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
	{
		Error_Handler();
	}

	for (;;)
	{
		if (HAL_GetTick() < 10000)
		{
			datalogging.write();
		}
		else
		{
			datalogging.read();
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		main_loop();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart == &huart3)
	{
		gnss.callback();
	}
}

extern "C"
{
void autopilot_main_c()
{
	autopilot_main();
}
}
