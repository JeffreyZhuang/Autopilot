#include "Flight_HAL/flight_hal.h"

void Flight_hal::init_rangefinder()
{
	cxof.setup();
}

void Flight_hal::read_rangefinder()
{
	Cxof_frame result;
	if (cxof.read(&result))
	{
		float flow_x = result.x - _plane->imu_gx * M_PI / 180.0f;
		float flow_y = result.y - _plane->imu_gy * M_PI / 180.0f;
		float flow_mag = sqrtf(flow_x*flow_x + flow_y*flow_y);
		_plane->rangefinder_dist = _plane->nav_airspeed / flow_mag;
		_plane->rangefinder_timestamp = get_time_us();

		printf("Flight_HAL Flow: %f %f\n", flow_x, flow_y);
	}
}
