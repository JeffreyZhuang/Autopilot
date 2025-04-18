#ifndef L1_CONTROLLER_H_
#define L1_CONTROLLER_H_

#include "lib/pi_control/pi_control.h"
#include "lib/utils/utils.h"
#include "hal.h"
#include "constants.h"
#include "module.h"
#include "params.h"
#include <math.h>
#include <cstdio>

class PositionControl : public Module
{
public:
	PositionControl(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<AHRS_data> _ahrs_sub;
	Subscriber<local_position_s> _local_pos_sub;
	Subscriber<Modes_data> _modes_sub;
	Subscriber<Telem_data> _telem_sub;
	Subscriber<waypoint_s> _waypoint_sub;
	Subscriber<RC_data> _rc_sub;
	Subscriber<time_s> _time_sub;

	Publisher<position_control_s> _position_control_pub;

	Modes_data _modes_data{};
	AHRS_data _ahrs_data{};
	local_position_s _local_pos;
	Telem_data _telem_data{};
	waypoint_s _waypoint{};
	position_control_s _position_control{};
	RC_data _rc_data{};
	time_s _time_data{};

	PI_control energy_balance_controller;
	PI_control total_energy_controller;

	float _total_energy_setpoint = 0;
	float _total_energy = 0;
	float _energy_balance_setpoint = 0;
	float _energy_balance = 0;
	float _d_setpoint = 0;

	void handle_manual_mode();
	void handle_auto_mode();
	void update_direct();
	void update_stabilized();
	void update_takeoff();
	void update_mission();
	void update_land();
	void update_flare();
	float calculate_altitude_setpoint(const float prev_north, const float prev_east, const float prev_down,
	  	  	  	  	  	 	 	 	  const float tgt_north, const float tgt_east, const float tgt_down);
	float l1_calculate_roll() const;
	float compute_along_track_distance(float start_n, float start_e, float end_n, float end_e,
			 	 	 	 	 	 	   float pos_n, float pos_e);
	float tecs_control_total_energy();
	float tecs_control_energy_balance();
	void tecs_calculate_energies(float target_vel_mps, float target_alt_m, float wb);
};

#endif /* L1_CONTROLLER_H_ */
