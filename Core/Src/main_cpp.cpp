#include "main_cpp.h"
#include "autopilot.h"

// Exclude Flight_HAL source folder from build if PITL is enabled
#define PITL_ENABLE true

Plane plane;

#if PITL_ENABLE
#include "pitl_hal.h"
Pitl_hal hal(&plane);
#else
#include "Flight_HAL/flight_hal.h"
Flight_hal hal(&plane);
#endif

Autopilot autopilot(&hal, &plane);

extern "C"
{
void main_c()
{
	autopilot.run();
}
}
