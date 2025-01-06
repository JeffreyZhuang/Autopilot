/*
 * guidance.h
 *
 *  Created on: Dec. 28, 2024
 *      Author: jeffr
 */

#ifndef GUIDANCE_H_
#define GUIDANCE_H_

#include "hal.h"

struct Setpoint {
	float n;
	float e;
	float d;
};

class Guidance
{
public:
	Guidance(HAL* hal, Plane* plane);

	void update();
private:
	HAL* _hal;
	Plane*_plane;

	Setpoint setpoints[10];
};

#endif /* GUIDANCE_H_ */
