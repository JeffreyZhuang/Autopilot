#include <flight_hal.h>

// Switch auto gen into classs like this: https://github.com/loveuav/Eigen-STM32-Demo/blob/main/Src/board.hpp

Flight_hal* Flight_hal::_instance = nullptr;

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

void Flight_hal::set_background_task(void (*task)())
{
	background_task = task;

	while (1)
	{
		background_task();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim7)
	{
		Flight_hal::main_task_callback();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart == &huart3)
	{
		Flight_hal::gnss_dma_complete();
	}
}

