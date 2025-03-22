#ifndef LIB_TECS_TECS_H_
#define LIB_TECS_TECS_H_

#include "Lib/PIControl/pi_control.h"
#include "Lib/Utils/utils.h"
#include "constants.h"
#include "plane.h"
#include "parameters.h"
#include "module.h"
#include <math.h>

class Tecs : public Module
{
public:
	Tecs(HAL* hal, Plane* plane);

	void update();

private:
	PI_control pitch_controller;
	PI_control throttle_controller;

	void update_takeoff();
	void update_mission();
	void update_land();
	void update_flare();
	void calculate_energies(float target_vel_mps, float target_alt_m, float wb);
	void control_pitch();
	void control_throttle();
};


#endif /* LIB_TECS_TECS_H_ */
