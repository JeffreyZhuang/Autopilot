#include "autopilot_main.h"
#include "autopilot.h"

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

	// Execute HAL on UART callback, so it will do PITL or Flight depending on whats included
}

extern "C"
{
	void autopilot_main_c()
	{
		autopilot_main();
	}
}
