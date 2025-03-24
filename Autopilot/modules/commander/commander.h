#ifndef MODULES_COMMANDER_COMMANDER_H_
#define MODULES_COMMANDER_COMMANDER_H_

#include "parameters.h"
#include "plane.h"
#include "hal.h"
#include "module.h"
#include <stdio.h>

class Commander : public Module
{
public:
	Commander(HAL* hal, Plane* plane);

	void update();

private:
	Plane::Subscription_handle ahrs_handle;

	void handle_flight_mode();
	void handle_manual_mode();
	void handle_auto_mode();
	void handle_switches();
	void update_config();
	void update_startup();
	void update_takeoff();
	void update_mission();
	void update_land();
	void update_flare();
	void update_touchdown();
};

#endif /* MODULES_COMMANDER_COMMANDER_H_ */
