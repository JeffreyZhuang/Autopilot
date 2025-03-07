#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_rangefinder()
{
	cxof.setup();
}

void Flight_hal::read_rangefinder()
{
	float flow_x = cxof.result.x - _plane->imu_gx;
	float flow_y = cxof.result.y - _plane->imu_gy;
	float flow_mag = sqrtf(flow_x*flow_x + flow_y*flow_y);
	_plane->rangefinder_dist = _plane->nav_airspeed / flow_mag;
	_plane->rangefinder_timestamp = get_time_us();
}
