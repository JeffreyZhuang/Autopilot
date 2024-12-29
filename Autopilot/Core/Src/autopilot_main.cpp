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
	autopilot.setup();

	for (;;)
	{
		autopilot.logging_loop();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim7)
	{
		autopilot.loop();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart == &huart3)
	{
		derived_hal.gnss_callback();
	}
}

extern "C"
{
void autopilot_main_c()
{
	autopilot_main();
}
}
