#ifndef MODULES_COMMANDER_COMMANDER_H_
#define MODULES_COMMANDER_COMMANDER_H_

#include <lib/hal/hal.h>
#include <lib/module/module.h>
#include "lib/parameters/params.h"
#include "lib/data_bus/data_bus.h"
#include <stdio.h>

class Commander : public Module
{
public:
	Commander(HAL* hal, DataBus* data_bus);

	void update() override;

private:
	Subscriber<AHRS_data> _ahrs_sub;
	Subscriber<local_position_s> _local_pos_sub;
	Subscriber<RC_data> _rc_sub;
	Subscriber<waypoint_s> _waypoint_sub;

	Publisher<Modes_data> _modes_pub;

	local_position_s _local_pos;
	AHRS_data _ahrs_data{};
	Modes_data _modes_data{};
	RC_data _rc_data{};
	waypoint_s _waypoint{};

	// Parameters
	float _takeoff_alt;
	float _flare_alt;

	void update_parameters();
	void poll_vehicle_data();

	void handle_flight_mode();
	void handle_auto_mode();
	void handle_switches();
	void update_config();
	void update_startup();
	void update_takeoff();
	void update_land();
};

#endif /* MODULES_COMMANDER_COMMANDER_H_ */
