#include "Autopilot_HAL/Autopilot_HAL.h"

void AutopilotHAL::set_main_task(void (*task)())
{
	main_task = task;

	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
}

void AutopilotHAL::execute_main_task()
{
	if (main_task)
	{
		main_task();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim7) // 100hz high priority interrupt
	{
		AutopilotHAL::main_task_callback();
	}
	else if (htim == &htim6) // 1hz low priority interrupt
	{
		AutopilotHAL::sd_interrupt_callback();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart == &huart2)
	{
		AutopilotHAL::of_dma_complete();
	}
	else if (huart == &huart3)
	{
		AutopilotHAL::gnss_dma_complete();
	}
	else if (huart == &huart4)
	{
		AutopilotHAL::rc_dma_complete();
	}
	else if (huart == &huart6)
	{
		AutopilotHAL::telemetry_dma_complete();
	}
}

void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	AutopilotHAL::usb_rx_callback(Buf, Len);
}
