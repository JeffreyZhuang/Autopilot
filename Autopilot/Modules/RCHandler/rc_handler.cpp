#include "rc_handler.h"

Rc_handler::Rc_handler(HAL* hal, Plane* plane)
{
	_hal = hal;
	_plane = plane;
}

void Rc_handler::rc_update()
{
	uint16_t rc_input[16];
	_hal->get_rc_input(rc_input, 16);

	for (int i = 0; i < 16; i++)
	{
		if (i == params.throttle_ch)
		{
			_plane->rc_in_norm[i] = map(rc_input[i], params.rc_in_min, params.rc_in_max, 0, 1);
		}
		else
		{
			_plane->rc_in_norm[i] = map(rc_input[i], params.rc_in_min, params.rc_in_max, -1, 1);
		}
	}

	_plane->manual_sw = _plane->rc_in_norm[params.manual_sw_ch] > 0.5;
	_plane->mode_sw = _plane->rc_in_norm[params.mode_sw_ch] > 0.5;
}
