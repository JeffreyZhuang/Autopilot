#include "modules/rc_handler/rc_handler.h"

Rc_handler::Rc_handler(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _modes_sub(data_bus->modes_node),
	  _rc_pub(data_bus->rc_node)
{
}

void Rc_handler::update()
{
	_modes_data = _modes_sub.get();

	if (_modes_data.system_mode != System_mode::CONFIG)
	{
		// Get RC input duty cycle
		uint16_t rc_input[NUM_CH];
		_hal->get_rc_input(rc_input, NUM_CH);

		// Convert to values from -1 to 1
		_rc_data.ail_norm = map(rc_input[AIL_CH], get_params()->rc_input.min_duty, get_params()->rc_input.max_duty, -1, 1);
		_rc_data.ele_norm = map(rc_input[ELE_CH], get_params()->rc_input.min_duty, get_params()->rc_input.max_duty, -1, 1);
		_rc_data.rud_norm = map(rc_input[RUD_CH], get_params()->rc_input.min_duty, get_params()->rc_input.max_duty, -1, 1);
		_rc_data.thr_norm = map(rc_input[THR_CH], get_params()->rc_input.min_duty, get_params()->rc_input.max_duty, 0, 1);

		uint16_t midpoint = get_params()->rc_input.min_duty + (get_params()->rc_input.max_duty - get_params()->rc_input.min_duty) / 2;
		_rc_data.man_sw = rc_input[MAN_CH] > midpoint;
		_rc_data.mod_sw = rc_input[MOD_CH] > midpoint;

		// TODO: Need more proper way to detect tx connected
		_rc_data.tx_conn = rc_input[THR_CH] > get_params()->rc_input.min_duty / 2;

		_rc_pub.publish(_rc_data);
	}
}
