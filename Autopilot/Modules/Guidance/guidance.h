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
	void init();
	void update();
	bool reached_wp(Waypoint wp);
	bool reached_last_wp();
private:
	HAL* _hal;
	Plane*_plane;
	float kP = 1;
	bool flare_initialized = false;
	uint64_t flare_start_time = 0;
	void handle_manual_mode();
	void handle_auto_mode();
	void update_mission();
	void update_landing();
	void update_flare();
};

#endif /* GUIDANCE_H_ */
