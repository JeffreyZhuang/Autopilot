/*
 * guidance.h
 *
 *  Created on: Dec. 28, 2024
 *      Author: jeffr
 */

#ifndef GUIDANCE_H_
#define GUIDANCE_H_

#include "hal.h"
#include "Lib/Utils/utils.h"
#include <math.h>

class Guidance
{
public:
	Guidance(HAL* hal, Plane* plane);

	void init();
	void update_mission();
	void update_landing();
	void update_flare();
private:
	HAL* _hal;
	Plane*_plane;

	float kP = 1;
};

#endif /* GUIDANCE_H_ */
