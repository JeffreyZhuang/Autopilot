#include "commander.h"

Commander::Commander(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

void Commander::update()
{
	switch (_plane->flightState)
	{
	case FlightState::TAKEOFF:
		if (_hal->get_time_us() > 10000000)
		{
			_plane->flightState = FlightState::LAND;
		}

		break;
	case FlightState::LAND:
		break;
	}
}
