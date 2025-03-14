#ifndef LIB_TECS_TECS_H_
#define LIB_TECS_TECS_H_

#include "Lib/Utils/utils.h"
#include "constants.h"
#include "plane.h"
#include "parameters.h"
#include <math.h>

class Tecs
{
public:
	Tecs(Plane* plane);

	void update();

private:
	Plane* _plane;

	void update_mission();
	void update_land();
	void update_flare();
	void calculate(float target_vel_mps, float target_alt_m, float wb);
};


#endif /* LIB_TECS_TECS_H_ */
