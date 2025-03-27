#ifndef L1_CONTROLLER_H_
#define L1_CONTROLLER_H_

#include "lib/utils/utils.h"
#include "hal.h"
#include "constants.h"
#include "parameters.h"
#include "module.h"
#include <math.h>
#include <cstdio>

class L1_controller : public Module
{
public:
	L1_controller(HAL* hal);

	void update() override;

private:
	Subscriber<AHRS_data> _ahrs_sub;
	Subscriber<Pos_est_data> _pos_est_sub;
	Subscriber<Modes_data> _modes_sub;
	Subscriber<Telem_data> _telem_sub;
	Subscriber<Navigator_data> _navigator_sub;
	Subscriber<RC_data> _rc_sub;
	Subscriber<Time_data> _time_sub;

	Publisher<L1_data> _l1_pub;

	Modes_data _modes_data;
	AHRS_data _ahrs_data;
	Pos_est_data _pos_est_data;
	Telem_data _telem_data;
	Navigator_data _navigator_data;
	L1_data _l1_data;
	RC_data _rc_data;
	Time_data _time_data;

	void handle_manual_mode();
	void handle_auto_mode();
	void update_stabilized();
	void update_mission();
	void update_flare();
	float calculate_altitude_setpoint(const float prev_north, const float prev_east,
	  	  	  	  	  	 	 	 	  const float tgt_north, const float tgt_east,
									  const Waypoint& prev_wp, const Waypoint& target_wp);
	float calculate_roll_setpoint(float lateral_accel) const;
	float compute_along_track_distance(float start_n, float start_e, float end_n, float end_e,
			 	 	 	 	 	 	   float pos_n, float pos_e);
	float distance(float n1, float e1, float n2, float e2);
};

#endif /* L1_CONTROLLER_H_ */
