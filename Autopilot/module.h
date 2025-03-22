#ifndef MODULE_H_
#define MODULE_H_

#include "plane.h"
#include "hal.h"

/**
 * Abstract module class
 */
class Module
{
public:
	Module(HAL* hal, Plane* plane);
	virtual void update() = 0;

private:
	HAL* _hal;
	Plane* _plane;
};


#endif /* MODULE_H_ */
