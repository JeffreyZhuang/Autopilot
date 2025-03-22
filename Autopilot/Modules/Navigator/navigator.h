#ifndef MODULES_NAVIGATOR_NAVIGATOR_H_
#define MODULES_NAVIGATOR_NAVIGATOR_H_

#include "plane.h"
#include "hal.h"

class Navigator
{
public:
	Navigator(HAL* hal, Plane* plane);
	void update();
private:
	Plane* _plane;
	HAL* _hal;
};


#endif /* MODULES_NAVIGATOR_NAVIGATOR_H_ */
