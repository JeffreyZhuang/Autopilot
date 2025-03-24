#include "modules/rc_handler/rc_handler.h"

Rc_handler::Rc_handler(HAL* hal, Plane* plane) : Module(hal, plane)
{
}

void Rc_handler::update()
{
	if (_plane->system_mode != Plane::System_mode::CONFIG)
	{
		// Get RC input duty cycle
		uint16_t rc_input[NUM_CH];
		_hal->get_rc_input(rc_input, NUM_CH);

		// Convert to values from -1 to 1
		_plane->rc_ail_norm = map(rc_input[AIL_CH], get_params()->rc_input.min_duty, get_params()->rc_input.max_duty, -1, 1);
		_plane->rc_ele_norm = map(rc_input[ELE_CH], get_params()->rc_input.min_duty, get_params()->rc_input.max_duty, -1, 1);
		_plane->rc_rud_norm = map(rc_input[RUD_CH], get_params()->rc_input.min_duty, get_params()->rc_input.max_duty, -1, 1);
		_plane->rc_thr_norm = map(rc_input[THR_CH], get_params()->rc_input.min_duty, get_params()->rc_input.max_duty, 0, 1);

		uint16_t midpoint = get_params()->rc_input.min_duty + (get_params()->rc_input.max_duty - get_params()->rc_input.min_duty) / 2;
		_plane->rc_man_sw = rc_input[MAN_CH] > midpoint;
		_plane->rc_mod_sw = rc_input[MOD_CH] > midpoint;

		_plane->tx_connected = rc_input[THR_CH] > get_params()->rc_input.min_duty / 2;
	}
}
