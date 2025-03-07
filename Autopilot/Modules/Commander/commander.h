#ifndef MODULES_COMMANDER_COMMANDER_H_
#define MODULES_COMMANDER_COMMANDER_H_

#include "parameters.h"
#include "plane.h"
#include "hal.h"

class Commander
{
public:
	Commander(HAL* hal, Plane* plane);
	void update();
private:
	HAL* _hal;
	Plane* _plane;
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
