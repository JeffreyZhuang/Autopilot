#ifndef LIB_TECS_TECS_H_
#define LIB_TECS_TECS_H_

#include <data_bus.h>
#include "lib/pi_control/pi_control.h"
#include "lib/utils/utils.h"
#include "constants.h"
#include "params.h"
#include "module.h"
#include <math.h>

class Tecs : public Module
{
public:
	Tecs(HAL* hal, Data_bus* data_bus);

	void update();

private:
	PI_control energy_balance_controller;
	PI_control total_energy_controller;
	float _total_energy_setpoint = 0;
	float _total_energy = 0;
	float _energy_balance_setpoint = 0;
	float _energy_balance = 0;

	Subscriber<Pos_est_data> _pos_est_sub;
	Subscriber<Modes_data> _modes_sub;
	Subscriber<Time_data> _time_sub;
	Subscriber<RC_data> _rc_sub;
	Subscriber<L1_data> _l1_sub;

	Publisher<TECS_data> _tecs_pub;

	Pos_est_data _pos_est_data{};
	Modes_data _modes_data{};
	Time_data _time_data{};
	RC_data _rc_data{};
	TECS_data _tecs_data{};
	L1_data _l1_data{};

	void handle_manual_mode();
	void handle_auto_mode();
	void update_direct();
	void update_stabilized();
	void update_takeoff();
	void update_mission();
	void update_land();
	void update_flare();
	void calculate_energies(float target_vel_mps, float target_alt_m, float wb);
	float control_energy_balance();
	float control_total_energy();
};


#endif /* LIB_TECS_TECS_H_ */
