#include "Flight_HAL/flight_hal.h"

void Flight_hal::start_main_task(void (*task)(void*), void* arg)
{
	main_task = task;
	main_task_arg = arg;

	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
	{
		Error_Handler();
	}
}

void Flight_hal::execute_main_task()
{
	if (main_task)
	{
		main_task(main_task_arg);
	}
}

void Flight_hal::start_background_task(void (*task)(void*), void* arg)
{
	background_task = task;

	while (1)
	{
		background_task(arg);
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
	if (huart == &huart2)
	{
		Flight_hal::of_dma_complete();
	}
	else if (huart == &huart3)
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

void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	Flight_hal::get_instance()->usb_rx_callback(Buf, Len);
}
