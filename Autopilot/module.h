#ifndef MODULE_H_
#define MODULE_H_

#include <data_bus.h>
#include "hal.h"

/**
 * Abstract module class
 */
class Module
{
public:
	Module(HAL* hal);
	virtual ~Module() {};

	virtual void update() = 0;

protected:
	HAL* _hal;
};

#endif /* MODULE_H_ */
