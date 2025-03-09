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
	void update(float target_vel_mps, float target_alt_m, float wb);
private:
	Plane* _plane;
};


#endif /* LIB_TECS_TECS_H_ */
