#ifndef L1_CONTROLLER_H_
#define L1_CONTROLLER_H_

#include "lib/utils/utils.h"
#include "hal.h"
#include "constants.h"
#include "parameters.h"
#include "module.h"
#include <math.h>
#include <cstdio>

class L1_controller : public Module
{
public:
	L1_controller(HAL* hal, Plane* plane);

	void update() override;

private:
	Subscription_handle ahrs_handle;

	void handle_auto_mode();
	void update_mission();
	void update_flare();
	float calculate_roll_setpoint(float lateral_accel) const;
	float compute_along_track_distance(float start_n, float start_e, float end_n, float end_e,
			 	 	 	 	 	 	   float pos_n, float pos_e);
	float distance(float n1, float e1, float n2, float e2);
};

#endif /* L1_CONTROLLER_H_ */
