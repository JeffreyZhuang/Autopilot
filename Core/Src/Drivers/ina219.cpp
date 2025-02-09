/*
 * ina219.cpp
 *
 *  Created on: Dec. 6, 2024
 *      Author: jeffr
 */

#include "ina219.h"

INA219::INA219(I2C_HandleTypeDef * hi2c, float shunt_resistance)
{
	_shunt_resistance = shunt_resistance;
	_hi2c = hi2c;
}

float INA219::read_voltage()
{
	return (float)((read_16(0x02) >> 3) * 4) * 0.001; // V
}

float INA219::read_current()
{
	return (float)read_16(0x01) * 0.01 * 0.001 / _shunt_resistance; // A
}

uint16_t INA219::read_16(uint8_t reg)
{
	uint8_t value[2];

	HAL_I2C_Mem_Read(_hi2c, 0x40<<1, reg, 1, value, 2, 1000);

	return ((value[0] << 8) | value[1]);
}
