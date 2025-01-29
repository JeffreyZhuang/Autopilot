#include "main_cpp.h"
#include "autopilot.h"

// Exclude Flight_HAL source folder from build if PITL is enabled
#define PITL_ENABLE true
#if PITL_ENABLE
	#include "pitl_hal.h"
	using Hal = Pitl_hal;
#else
	#include "Flight_HAL/flight_hal.h"
	using Hal = Flight_hal;
#endif

void main_cpp()
{
	Plane plane;
	Hal hal(&plane);
	Autopilot autopilot(&hal, &plane);

	autopilot.run();
}

extern "C"
{
	void main_c()
	{
		main_cpp();
	}
}
