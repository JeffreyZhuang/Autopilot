#include "pitl_hal.h"

void Pitl_hal::set_elevator(float deg)
{
	pitl_tx_packet.elevator = deg;
}

void Pitl_hal::set_rudder(float deg)
{
	pitl_tx_packet.aileron = deg;
}

void Pitl_hal::set_throttle(float throttle)
{
	pitl_tx_packet.throttle = throttle;
}
