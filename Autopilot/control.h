#ifndef CONTROL_H_
#define CONTROL_H_

#include <math.h>
#include "hal.h"
#include "pid.h"

// Calculates and sends commands to servos
class Control
{
public:
	Control(HAL* hal, Plane* plane, float dt);

	void update_manual();
	void update_takeoff();
	void update_mission();
	void update_land();
	void update_stabalize();

private:
	HAL* _hal;
	Plane* _plane;

	float _dt;

	PID roll_controller;
	PID pitch_controller;
	PID hdg_controller;
	PID alt_controller;
	PID speed_controller;

	float clamp(float n, float min, float max);
};

#endif /* CONTROL_H_ */
