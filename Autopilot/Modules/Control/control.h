#ifndef CONTROL_H_
#define CONTROL_H_

#include "Lib/TECS/tecs.h"
#include "Lib/PID/pid.h"
#include "Lib/Utils/utils.h"
#include "hal.h"
#include <math.h>
#include <cstdio>

// Calculates and sends commands to servos
class Control
{
public:
	Control(HAL* hal, Plane* plane);

	void update();

private:
	HAL* _hal;
	Plane* _plane;
	PID roll_controller;
	PID pitch_controller;
	PID hdg_controller;
	PID alt_controller;
	PID speed_controller;
	Tecs _tecs;

	void handle_manual_mode();
	void handle_auto_mode();

	void update_direct();
	void update_stabilized();
	void update_takeoff();
	void update_mission();
	void update_land();
	void update_flare();
	void update_touchdown();
};

#endif /* CONTROL_H_ */
