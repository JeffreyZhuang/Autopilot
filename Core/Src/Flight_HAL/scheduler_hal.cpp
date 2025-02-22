#include "Flight_HAL/flight_hal.h"

// Switch auto gen into classs like this: https://github.com/loveuav/Eigen-STM32-Demo/blob/main/Src/board.hpp

void Flight_hal::set_main_task(void (*task)())
{
	main_task = task;

	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
	{
		Error_Handler();
	}
}

void Flight_hal::execute_main_task()
{
	if (main_task)
	{
		main_task();
	}
}

float Flight_hal::get_main_dt() const
{
	return 0.01;
}

void Flight_hal::set_background_task(void (*task)())
{
	background_task = task;

	while (1)
	{
		background_task();
	}
}

#if !PITL_ENABLE
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim7)
	{
		Flight_hal::main_task_callback();
	}
}

__weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart == &huart3)
	{
		Flight_hal::gnss_dma_complete();
	}
	else if (huart == &huart4)
	{
		Flight_hal::rc_dma_complete();
	}
	else if (huart == &huart6)
	{
		Flight_hal::telemetry_dma_complete();
	}
}
#endif
