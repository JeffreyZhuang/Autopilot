#include <flight_hal.h>
#include "plane.h"
#include "pitl_hal.h"
#include <scheduler.h>
#include "autopilot.h"

#define PITL_ENABLE false

Plane plane;

#if PITL_ENABLE
Pitl_hal pitl_hal;
Autopilot autopilot(&pitl_hal, &plane);
#else
Derived_hal derived_hal(&plane);
Autopilot autopilot(&derived_hal, &plane);
#endif

void main_cpp()
{
	autopilot.init();

	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
	{
		Error_Handler();
	}

	while (1)
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
