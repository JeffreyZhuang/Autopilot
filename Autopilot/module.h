#ifndef MODULE_H_
#define MODULE_H_

#include <lib/data_bus/data_bus.h>
#include "hal.h"

/**
 * Abstract module class
 */
class Module
{
public:
	Module(HAL* hal, DataBus* data_bus);
	virtual ~Module() {};

	virtual void update() = 0;

protected:
	HAL* _hal;
	DataBus* _data_bus;
};

#endif /* MODULE_H_ */
