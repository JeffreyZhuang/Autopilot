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
	virtual ~Module() {};

	virtual void update() = 0;

protected:
	HAL* _hal;
	Plane* _plane;
};

#endif /* MODULE_H_ */
