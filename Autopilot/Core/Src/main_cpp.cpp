#include "main_cpp.h"
#include "autopilot.h"

// Use abstract AHRS class in Autopilot folder
// Put different types of AHRS in Library folder (e.g. madwick_ahrs, mahony_ahrs, kalman_ahrs)
// Flight_hal or PITL_hal chooses which AHRS to use
// Therefore HAL can run PITL instead of Flight

// TODO:
// How to generate trajectory? Don't need trajectory generation, just go straight to nearest waypoint or selected waypoint?
// How to store waypoints? Sd or radio?
// What format on micro sd?
// Just write structs. Each line on file is a waypoint struct.
// Different beta for mag and imu
//

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
	autopilot.init();
}
}
