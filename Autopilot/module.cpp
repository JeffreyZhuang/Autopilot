#include "module.h"

Module::Module(HAL* hal, DataBus* data_bus)
{
	_hal = hal;
	_data_bus = data_bus;
}
