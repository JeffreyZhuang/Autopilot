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
		float alt_weight;
		float pitch_gain;
		float throttle_gain;
	};

	TECS() = default;
	~TECS() = default;

	void update(float alt_m, float vel_mps, float target_alt_m, float target_vel_mps);

	void set_min_spd(float min_spd) { _param.min_spd = min_spd; };
	void set_max_spd(float max_spd) { _param.max_spd = max_spd; };
	void set_alt_weight(float alt_weight) { _param.alt_weight = alt_weight; };
	void set_pitch_gain(float pitch_gain) { _param.pitch_gain = pitch_gain; };
	void set_throttle_gain(float throttle_gain) { _param.throttle_gain = throttle_gain; };

	float get_pitch_setpoint() { return _pitch_setpoint; };
	float get_throttle_setpoint() { return _throttle_setpoint; };

private:
	struct ControlValues {
		float setpoint;
		float estimate;
	};

	struct SpecificEnergies {
		ControlValues kinetic;
		ControlValues potential;
	};

	struct State {
		float alt_m;
		float vel_mps;
	};

	struct Setpoint {
		float target_alt_m;
		float target_vel_mps;
	};

	SpecificEnergies calc_specific_energies(const State state, const Setpoint setpoint, const Param param);

	ControlValues calc_energy_balance(const SpecificEnergies specific_energies, const Param param);
	void calc_pitch_control(const SpecificEnergies specific_energies, const Param param);

	ControlValues calc_total_energy(const SpecificEnergies specific_energies, const Param param);
	void calc_throttle_control(const SpecificEnergies specific_energies, const Param param);

	Param _param{0};

	float _pitch_setpoint = 0;
	float _throttle_setpoint = 0;
};

#endif /* LIB_TECS_TECS_H_ */
