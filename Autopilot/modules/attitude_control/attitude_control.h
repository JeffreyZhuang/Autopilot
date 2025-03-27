#ifndef CONTROL_H_
#define CONTROL_H_

#include "lib/pi_control/pi_control.h"
#include "lib/utils/utils.h"
#include "hal.h"
#include "module.h"
#include "parameters.h"
#include <math.h>
#include <cstdio>

class Attitude_control : public Module
{
public:
	Attitude_control(HAL* hal);

	void update();

private:
	PI_control roll_controller;
	PI_control pitch_controller;

	Plane::Subscription_handle _ahrs_handle;
	Plane::Subscription_handle _att_setpoint_handle;

	Plane::AHRS_data _ahrs_data;
	Plane::Att_setpoint_data _att_setpoint_data;

	void handle_manual_mode();
	void handle_auto_mode();

	void update_direct();
	void update_stabilized();

	void update_takeoff();
	void update_mission();
	void update_touchdown();

	void control_roll_ptch();
	void control_roll_ptch_no_integral();
};

#endif /* CONTROL_H_ */
