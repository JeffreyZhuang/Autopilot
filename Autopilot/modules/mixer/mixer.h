#ifndef MODULES_MIXER_MIXER_H_
#define MODULES_MIXER_MIXER_H_

#include <lib/hal/hal.h>
#include <lib/module/module.h>
#include "lib/parameters/params.h"
#include "lib/utils/utils.h"

class Mixer : public Module
{
public:
	Mixer(HAL* hal, DataBus* data_bus);

	void update() override;

private:
	Subscriber<Modes_data> _modes_sub;
	Subscriber<position_control_s> _position_control_sub;
	Subscriber<Ctrl_cmd_data> _ctrl_cmd_sub;

	Publisher<HITL_output_data> _hitl_output_pub;

	Modes_data _modes_data{};
	Ctrl_cmd_data _ctrl_cmd_data{};
	position_control_s _position_control{};

	uint16_t _elevator_duty = 0;
	uint16_t _rudder_duty = 0;
	uint16_t _throttle_duty = 0;

	// Parameters
	int32_t _pwm_min_ele;
	int32_t _pwm_max_ele;
	int32_t _pwm_min_rud;
	int32_t _pwm_max_rud;
	int32_t	_pwm_min_thr;
	int32_t _pwm_max_thr;
	int32_t _rev_ele;
	int32_t _rev_rud;

	void parameters_update();

	void update_config();
	void update_startup();
	void update_flight();
};

#endif /* MODULES_MIXER_MIXER_H_ */
