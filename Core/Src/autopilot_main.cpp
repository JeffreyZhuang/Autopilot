#include "autopilot_main.h"
#include "autopilot.h"
#include "Flight_HAL/flight_hal.h"

void autopilot_main_c()
{
	Flight_hal hal;
	Autopilot autopilot(&hal);
	autopilot.setup();
}
