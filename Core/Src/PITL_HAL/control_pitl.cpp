#include "pitl_hal.h"

void Pitl_hal::set_elevator_duty(uint16_t duty)
{
	printf("%d\n", duty);
	pitl_tx_packet.elevator = map(duty, ELEVATOR_MIN_DUTY, ELEVATOR_MAX_DUTY, -1, 1);
}

void Pitl_hal::set_rudder_duty(uint16_t duty)
{
	pitl_tx_packet.aileron = map(duty, AILERON_MIN_DUTY, AILERON_MAX_DUTY, -1, 1);
}

void Pitl_hal::set_throttle_duty(uint16_t duty)
{
	pitl_tx_packet.throttle = map(duty, THROTTLE_MIN_DUTY, THROTTLE_MAX_DUTY, 0, 1);;
}
