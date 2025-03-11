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
};

#endif /* GUIDANCE_H_ */
