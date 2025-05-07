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
	Module(HAL* hal, Data_bus* data_bus);
	virtual ~Module() {};

	virtual void update() = 0;

protected:
	HAL* _hal;
	Data_bus* _data_bus;
};

#endif /* MODULE_H_ */
