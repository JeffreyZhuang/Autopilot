#ifndef L1_CONTROLLER_H_
#define L1_CONTROLLER_H_

#include "lib/tecs/tecs.h"
#include "lib/constants/constants.h"
#include "lib/hal/hal.h"
#include "lib/module/module.h"
#include "lib/parameters/params.h"
#include "lib/pi_control/pi_control.h"
#include "lib/utils/utils.h"
#include "lib/mission/mission.h"
#include "lib/l1_control/l1_control.h"
#include <math.h>
#include <cstdio>

class PositionControl : public Module
{
public:
	PositionControl(HAL* hal, DataBus* data_bus);

	void update() override;

private:
	uint64_t _last_time = 0;
	float _dt = 0;

	Subscriber<AHRS_data> _ahrs_sub;
	Subscriber<local_position_s> _local_pos_sub;
	Subscriber<Modes_data> _modes_sub;
	Subscriber<waypoint_s> _waypoint_sub;
	Subscriber<RC_data> _rc_sub;

	Publisher<position_control_s> _position_control_pub;

	Modes_data _modes_data{};
	AHRS_data _ahrs_data{};
	local_position_s _local_pos;
	waypoint_s _waypoint{};
	position_control_s _position_control{};
	RC_data _rc_data{};

	TECS _tecs;
	L1Control _l1_control;

	float _d_setpoint = 0;

	void update_parameters();

	void handle_manual_mode();
	void update_direct();
	void update_stabilized();

	void handle_auto_mode();
	void update_takeoff();
	void update_mission();
	void update_land();
	void update_flare();

	float l1_calculate_roll() const;
	float compute_along_track_distance(float start_n, float start_e, float end_n, float end_e,
			 	 	 	 	 	 	   float pos_n, float pos_e);
};

#endif /* L1_CONTROLLER_H_ */
