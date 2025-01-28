/*
 * ina219.hpp
 *
 *  Created on: Dec. 6, 2024
 *      Author: jeffr
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"

class INA219
{
public:
	INA219(I2C_HandleTypeDef* hi2c, float shunt_resistance);

	float read_voltage();
	float read_current();
private:
	uint16_t read_16(uint8_t register);

	float _shunt_resistance;
	I2C_HandleTypeDef* _hi2c;
};

#endif /* INC_INA219_H_ */
