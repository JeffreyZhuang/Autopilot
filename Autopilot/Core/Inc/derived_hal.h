#ifndef INC_DERIVED_HAL_H_
#define INC_DERIVED_HAL_H_

#include "hal.h"
#include "plane.h"

class Derived_hal : public HAL
{
public:
	Derived_hal(Plane * plane);
	void setup();
	void poll();
private:
	Plane * _plane;
};

#endif /* INC_DERIVED_HAL_H_ */
