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
	enum class LandingState {
		LOITER,
		GLIDESLOPE,
		FLARE
	};

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

	// Time
	uint64_t _last_time = 0;
	float _dt = 0;

	// Landing
	LandingState _landing_state = LandingState::LOITER;
	float _flare_z_setpoint = 0;

	// Parameters
	float _roll_limit;
	float _pitch_limit;
	float _takeoff_pitch;
	float _cruise_speed;
	float _landing_speed;
	float _acceptance_radius;
	float _flare_sink_rate;
	float _flare_alt;

	void poll_vehicle_data();
	void update_time();
	void update_parameters();
	void publish_status();

	void handle_manual_mode();
	void update_direct();
	void update_stabilized();

	void handle_auto_mode();
	void update_takeoff();

	void update_mission();
	void update_mission_waypoint();
	void update_mission_loiter();

	void update_land();
	void update_land_loiter();
	void update_land_glideslope();
	void update_land_flare();
	void reset_landing_state();

	float compute_along_track_distance(float start_n, float start_e, float end_n, float end_e,
			 	 	 	 	 	 	   float pos_n, float pos_e);
	void move_point(float north, float east, float distance, float bearing_deg,
					float *new_north, float *new_east);
};

#endif /* L1_CONTROLLER_H_ */
