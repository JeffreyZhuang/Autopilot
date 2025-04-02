#ifndef MODULES_MIXER_MIXER_H_
#define MODULES_MIXER_MIXER_H_

#include "lib/utils/utils.h"
#include "hal.h"
#include "module.h"
#include "params.h"

class Mixer : public Module
{
public:
	Mixer(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<Modes_data> _modes_sub;
	Subscriber<TECS_data> _tecs_sub;
	Subscriber<Ctrl_cmd_data> _ctrl_cmd_sub;
	Subscriber<Telem_data> _telem_sub;

	Publisher<HITL_output_data> _hitl_output_pub;

	Modes_data _modes_data{};
	Ctrl_cmd_data _ctrl_cmd_data{};
	TECS_data _tecs_data{};
	Telem_data _telem_data{};

	uint16_t _elevator_duty = 0;
	uint16_t _rudder_duty = 0;
	uint16_t _throttle_duty = 0;

	void update_config();
	void update_startup();
	void update_flight();
};

#endif /* MODULES_MIXER_MIXER_H_ */
