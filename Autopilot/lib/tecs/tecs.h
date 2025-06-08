#ifndef LIB_TECS_TECS_H_
#define LIB_TECS_TECS_H_

#include "lib/constants/constants.h"
#include "lib/utils/utils.h"
#include <math.h>

class TECS
{
public:
	struct Param {
		float min_spd;
		float max_spd;
	};

	TECS() = default;
	~TECS() = default;

	void set_min_spd(float min_spd) { _param.min_spd = min_spd; };
	void set_max_spd(float max_spd) { _param.max_spd = max_spd; };

private:
	void calc_specific_energies(float alt_m, float vel_mps, float target_vel_mps, float target_alt_m, float wb);

	Param _param{0};

	float _total_energy_setpoint = 0;
	float _total_energy = 0;
	float _energy_balance_setpoint = 0;
	float _energy_balance = 0;
};

#endif /* LIB_TECS_TECS_H_ */
