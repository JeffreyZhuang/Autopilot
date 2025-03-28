#include "module.h"

Module::Module(HAL* hal, Data_bus* data_bus)
{
	_hal = hal;
	_data_bus = data_bus;
}
