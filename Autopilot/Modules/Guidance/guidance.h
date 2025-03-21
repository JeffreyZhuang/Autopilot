#ifndef GUIDANCE_H_
#define GUIDANCE_H_

#include "Lib/Utils/utils.h"
#include "hal.h"
#include "constants.h"
#include "parameters.h"
#include <math.h>
#include <cstdio>

class Guidance
{
public:
	Guidance(HAL* hal, Plane* plane);
	void update();

private:
	HAL* _hal;
	Plane*_plane;

	void handle_auto_mode();
	void update_mission();
	void update_flare();

	float compute_along_track_distance(float start_n, float start_e, float end_n, float end_e,
			 	 	 	 	 	 	   float pos_n, float pos_e);
	float distance(float n1, float e1, float n2, float e2);
	float normalize_heading(float heading);
};

#endif /* GUIDANCE_H_ */
