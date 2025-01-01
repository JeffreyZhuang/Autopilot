#include "main_cpp.h"
#include "flight_hal.h"
#include "pitl_hal.h"
#include "autopilot.h"

#define PITL_ENABLE true

Plane plane;

#if PITL_ENABLE
Pitl_hal hal(&plane);
#else
Flight_hal hal(&plane);
#endif

Autopilot autopilot(&hal, &plane);

void main_cpp()
{
	autopilot.init();
}

extern "C"
{
void main_c()
{
	main_cpp();
}
}
