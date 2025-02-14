/*
 * servos.h
 *
 *  Created on: Feb 8, 2025
 *      Author: jeffr
 */

#ifndef INC_DRIVERS_SERVO_H_
#define INC_DRIVERS_SERVO_H_

#include "stm32f4xx_hal.h"

class Servo
{
public:
	Servo(TIM_HandleTypeDef* tim, uint32_t channel);
	void init();
	void set_angle(uint8_t deg);
	void set_period(uint16_t us)
private:
	TIM_HandleTypeDef* _tim;
	uint32_t _channel;
};

#endif /* INC_DRIVERS_SERVO_H_ */
