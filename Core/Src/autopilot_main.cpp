#include "autopilot_main.h"
#include "autopilot.h"

// Exclude Flight_HAL source folder from build if PITL is enabled
#if PITL_ENABLE
#include "pitl_hal.h"
using Hal = Pitl_hal;
#else
#include "Flight_HAL/flight_hal.h"
using Hal = Flight_hal;
#endif

void autopilot_main()
{
	Plane plane;
	Hal hal(&plane);
	Autopilot autopilot(&hal, &plane);
	autopilot.setup();
}

extern "C"
{
	void autopilot_main_c()
	{
		autopilot_main();
	}
}
