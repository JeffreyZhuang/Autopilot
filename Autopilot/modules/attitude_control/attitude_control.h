#ifndef CONTROL_H_
#define CONTROL_H_

#include "lib/pi_control/pi_control.h"
#include "lib/utils/utils.h"
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
	PI_control roll_controller;
	PI_control pitch_controller;

	Subscriber<AHRS_data> _ahrs_sub;
	Subscriber<L1_data> _l1_sub;
	Subscriber<TECS_data> _tecs_sub;
	Subscriber<Modes_data> _modes_sub;
	Subscriber<RC_data> _rc_sub;
	Subscriber<Time_data> _time_sub;

	Publisher<Ctrl_cmd_data> _ctrl_cmd_pub;

	AHRS_data _ahrs_data;
	RC_data _rc_data;
	L1_data _l1_data;
	TECS_data _tecs_data;
	Modes_data _modes_data;
	Ctrl_cmd_data _ctrl_cmd_data;
	Time_data _time_data;

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
