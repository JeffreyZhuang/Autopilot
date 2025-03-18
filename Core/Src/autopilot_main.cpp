#include "autopilot_main.h"
#include "autopilot.h"
#include "Flight_HAL/flight_hal.h"

void autopilot_main_c()
{
	Plane plane;
	Flight_hal hal(&plane);
	Autopilot autopilot(&hal, &plane);
	autopilot.setup();
}
