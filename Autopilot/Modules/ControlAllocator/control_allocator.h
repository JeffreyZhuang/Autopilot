/*
 * control_allocator.h
 *
 *  Created on: Feb. 14, 2025
 *      Author: jeffr
 */

#ifndef MODULES_CONTROLALLOCATOR_CONTROL_ALLOCATOR_H_
#define MODULES_CONTROLALLOCATOR_CONTROL_ALLOCATOR_H_

#include "Lib/Utils/utils.h"
#include "hal.h"
#include "parameters.h"

// The attitude controller works using a cascaded loop method. The outer loop computes the error between the attitude setpoint and the estimated attitude that, multiplied by a gain (P controller), generates a rate setpoint. The inner loop then computes the error in rates and uses a PI (proportional + integral) controller to generate the desired angular acceleration.

class Control_allocator
{
public:
	Control_allocator(HAL* hal, Plane* plane);

	void update();
private:
	Plane* _plane;
	HAL* _hal;

	uint16_t _elevator_duty = 0;
	uint16_t _aileron_duty = 0;
	uint16_t _throttle_duty = 0;

	void publish_actuator_controls();
};

#endif /* MODULES_CONTROLALLOCATOR_CONTROL_ALLOCATOR_H_ */
