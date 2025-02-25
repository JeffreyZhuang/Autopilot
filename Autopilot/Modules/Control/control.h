#ifndef CONTROL_H_
#define CONTROL_H_

#include "Modules/TECS/tecs.h"
#include "Lib/PID/pid.h"
#include "Lib/Utils/utils.h"
#include "hal.h"
#include <math.h>

// Calculates and sends commands to servos
class Control
{
public:
	Control(HAL* hal, Plane* plane, float dt);

	void update_manual();
	void update_stabilized();
	void update_takeoff();
	void update_mission();
	void update_land();
	void update_flare();

private:
	HAL* _hal;
	Plane* _plane;
	PID roll_controller;
	PID pitch_controller;
	PID hdg_controller;
	PID alt_controller;
	PID speed_controller;
	Tecs _tecs;
	float _dt;
};

#endif /* CONTROL_H_ */
