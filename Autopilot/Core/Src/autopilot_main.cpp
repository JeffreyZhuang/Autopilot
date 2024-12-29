/*
 * autopilot_main.cpp
 *
 *  Created on: Dec. 6, 2024
 *      Author: jeffr
 */

#include <autopilot_main.h>
#include "plane.h"
#include <derived_hal.h>
#include "autopilot.h"

Plane plane;
Derived_hal derived_hal(&plane);
Autopilot autopilot(&derived_hal, &plane);

/**
 * Scheduler
 */
void autopilot_main()
{
	autopilot.init();

	for (;;)
	{
		autopilot.logger_task();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim7)
	{
		autopilot.main_task();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart == &huart3)
	{
		derived_hal.gnss_dma_complete();
	}
}

extern "C"
{
void autopilot_main_c()
{
	autopilot_main();
}
}
