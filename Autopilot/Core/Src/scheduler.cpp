#include "plane.h"
#include <derived_hal.h>
#include <scheduler.h>
#include "autopilot.h"

Plane plane;
Derived_hal derived_hal(&plane);
Autopilot autopilot(&derived_hal, &plane);

void main_cpp()
{
	autopilot.init();

	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
	{
		Error_Handler();
	}

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
void main_c()
{
	main_cpp();
}
}
