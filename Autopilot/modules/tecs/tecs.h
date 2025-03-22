#ifndef LIB_TECS_TECS_H_
#define LIB_TECS_TECS_H_

#include "lib/pi_control/pi_control.h"
#include "lib/utils/utils.h"
#include "constants.h"
#include "plane.h"
#include "parameters.h"
#include "module.h"
#include <math.h>

class Tecs : public Module
{
public:
	Tecs(HAL* hal, Plane* plane);

	void update();

private:
	PI_control energy_balance_controller;
	PI_control total_energy_controller;

	void handle_manual_mode();
	void handle_auto_mode();
	void update_direct();
	void update_stabilized();
	void update_takeoff();
	void update_mission();
	void update_land();
	void update_flare();
	void update_touchdown();
	void calculate_energies(float target_vel_mps, float target_alt_m, float wb);
	void control_energy_balance();
	void control_total_energy();
};


#endif /* LIB_TECS_TECS_H_ */
