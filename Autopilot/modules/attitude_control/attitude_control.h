#ifndef CONTROL_H_
#define CONTROL_H_

#include <lib/constants/constants.h>
#include <lib/hal/hal.h>
#include <lib/module/module.h>
#include "lib/pi_control/pi_control.h"
#include "lib/utils/utils.h"
#include "lib/parameters/params.h"
#include <math.h>
#include <cstdio>

class AttitudeControl : public Module
{
public:
	AttitudeControl(HAL* hal, DataBus* data_bus);

	void update() override;

private:
	uint64_t _last_time = 0;
	float _dt = 0;

	PI_control roll_controller;
	PI_control pitch_controller;

	Subscriber<AHRS_data> _ahrs_sub;
	Subscriber<position_control_s> _position_control_sub;
	Subscriber<Modes_data> _modes_sub;
	Subscriber<RC_data> _rc_sub;

	Publisher<Ctrl_cmd_data> _ctrl_cmd_pub;

	AHRS_data _ahrs_data;
	RC_data _rc_data;
	position_control_s _position_control;
	Modes_data _modes_data;
	Ctrl_cmd_data _ctrl_cmd_data;

	// Parameters
	float _roll_kp;
	float _roll_ki;
	float _ptch_kp;
	float _ptch_ki;

	void update_time();
	void update_parameters();
	void poll_vehicle_data();

	void handle_manual_mode();
	void update_direct();
	void update_stabilized();
	void handle_auto_mode();

	void publish_status();

	void control_roll_ptch();
};

#endif /* CONTROL_H_ */
