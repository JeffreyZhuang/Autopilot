/*
 * guidance.h
 *
 *  Created on: Dec. 28, 2024
 *      Author: jeffr
 */

#ifndef GUIDANCE_H_
#define GUIDANCE_H_

#include "hal.h"
#include <math.h>

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

	static constexpr int num_waypoints = 1;
	Waypoint waypoints[num_waypoints];
};

#endif /* GUIDANCE_H_ */
