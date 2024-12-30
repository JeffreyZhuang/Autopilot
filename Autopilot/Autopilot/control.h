#ifndef CONTROL_H_
#define CONTROL_H_

#include "hal.h"
#include "pid.h"

class Control
{
public:
	Control(HAL* hal, Plane* plane);
private:
	HAL* _hal;
	Plane* _plane;

	PID roll_controller;
	PID pitch_controller;
	PID yaw_controller;
	PID alt_controller;
	PID hdg_controller;
};

#endif /* CONTROL_H_ */
