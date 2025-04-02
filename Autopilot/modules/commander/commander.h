#ifndef MODULES_COMMANDER_COMMANDER_H_
#define MODULES_COMMANDER_COMMANDER_H_

#include <data_bus.h>
#include "hal.h"
#include "module.h"
#include "params.h"
#include <stdio.h>

class Commander : public Module
{
public:
	Commander(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<AHRS_data> _ahrs_sub;
	Subscriber<Pos_est_data> _pos_est_sub;
	Subscriber<RC_data> _rc_sub;
	Subscriber<Telem_data> _telem_sub;
	Subscriber<Navigator_data> _navigator_sub;

	Publisher<Modes_data> _modes_pub;

	Pos_est_data _pos_est_data{};
	AHRS_data _ahrs_data{};
	Modes_data _modes_data{};
	RC_data _rc_data{};
	Navigator_data _navigator_data{};
	Telem_data _telem_data{};

	void handle_flight_mode();
	void handle_auto_mode();
	void handle_switches();
	void update_config();
	void update_startup();
	void update_takeoff();
	void update_mission();
	void update_land();
};

#endif /* MODULES_COMMANDER_COMMANDER_H_ */
