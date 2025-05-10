#ifndef CONTROL_H_
#define CONTROL_H_

#include "lib/pi_control/pi_control.h"
#include "lib/utils/utils.h"
#include "lib/parameters/params.h"
#include "constants.h"
#include "hal.h"
#include "module.h"
#include <math.h>
#include <cstdio>

class Attitude_control : public Module
{
public:
	Attitude_control(HAL* hal, Data_bus* data_bus);

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
