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

	void update();
private:
	HAL* _hal;
	Plane* _plane;
	PID roll_controller;
	PID pitch_controller;
	PID alt_controller;
	PID hdg_controller;

	float _dt;
};

#endif /* CONTROL_H_ */
