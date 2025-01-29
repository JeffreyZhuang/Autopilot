/*
 * guidance.h
 *
 *  Created on: Dec. 28, 2024
 *      Author: jeffr
 */

#ifndef GUIDANCE_H_
#define GUIDANCE_H_

#include "hal.h"

struct Waypoint {
	float n;
	float e;
	float d;
};

class Guidance
{
public:
	Guidance(HAL* hal, Plane* plane);

	void init();
	void update();
private:
	HAL* _hal;
	Plane*_plane;

	Waypoint waypoints[3] = {{0, 0, -100},
							{100, 0, -50},
							{150, 10, -50}};
	int waypoint_idx = 0;
};

#endif /* GUIDANCE_H_ */
