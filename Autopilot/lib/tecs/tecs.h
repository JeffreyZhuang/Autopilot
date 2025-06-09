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
		float min_pitch;
		float max_pitch;
		float pitch_gain;
		float pitch_integral_gain;
		float min_throttle;
		float max_throttle;
		float throttle_trim;
		float throttle_gain;
		float throttle_integral_gain;
	};

	TECS() = default;
	~TECS() = default;

	void update(float alt_m, float vel_mps, float target_alt_m, float target_vel_mps, float dt);

	void set_param(Param param) { _param = param; };
	void set_min_spd(float min_spd) { _param.min_spd = min_spd; };
	void set_max_spd(float max_spd) { _param.max_spd = max_spd; };
	void set_alt_weight(float alt_weight) { _param.alt_weight = alt_weight; };
	void set_min_pitch(float min_pitch) { _param.min_pitch = min_pitch; };
	void set_max_pitch(float max_pitch) { _param.max_pitch = max_pitch; };
	void set_pitch_gain(float pitch_gain) { _param.pitch_gain = pitch_gain; };
	void set_pitch_integral_gain(float pitch_integral_gain) { _param.pitch_integral_gain = pitch_integral_gain; };
	void set_min_throttle(float min_throttle) { _param.min_throttle = min_throttle; };
	void set_max_throttle(float max_throttle) { _param.max_throttle = max_throttle; };
	void set_throttle_trim(float throttle_trim) { _param.throttle_trim = throttle_trim; };
	void set_throttle_gain(float throttle_gain) { _param.throttle_gain = throttle_gain; };
	void set_throttle_integral_gain(float throttle_integral_gain) { _param.throttle_integral_gain = throttle_integral_gain; };

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
	void calc_pitch_control(float dt, const SpecificEnergies specific_energies, const Param param);

	ControlValues calc_total_energy(const SpecificEnergies specific_energies, const Param param);
	void calc_throttle_control(float dt, const SpecificEnergies specific_energies, const Param param);

	Param _param{0};

	// State
	float _pitch_integ_state = 0;
	float _throttle_integ_state = 0;

	// Output
	float _pitch_setpoint = 0;
	float _throttle_setpoint = 0;
};

#endif /* LIB_TECS_TECS_H_ */
