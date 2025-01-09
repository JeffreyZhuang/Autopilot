#ifndef CONTROL_H_
#define CONTROL_H_

#include <math.h>
#include "hal.h"
#include "pid.h"

// Calculates and sends commands to servos
class Control
{
public:
	Control(HAL* hal, Plane* plane);

	void update();
private:
	HAL* _hal;
	Plane* _plane;

	float dt = 0.02;

	PID roll_controller;
	PID pitch_controller;
	PID yaw_controller;
	PID alt_controller;
	PID hdg_controller;
};

#endif /* CONTROL_H_ */
