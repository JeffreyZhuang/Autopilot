#include "autopilot_main.h"
#include "autopilot.h"
#include "Flight_HAL/flight_hal.h"

void autopilot_main_c()
{
	Data_bus data_bus;
	Flight_hal hal(&data_bus);
	Autopilot autopilot(&hal, &data_bus);
	autopilot.setup();
}
