#ifndef MODULES_RC_HANDLER_RC_HANDLER_H_
#define MODULES_RC_HANDLER_RC_HANDLER_H_

#include <data_bus.h>
#include <lib/utils/utils.h>
#include "hal.h"
#include "module.h"
#include "params.h"
#include <stdio.h>

// Channels
constexpr uint8_t AIL_CH = 0;
constexpr uint8_t ELE_CH = 1;
constexpr uint8_t THR_CH = 2;
constexpr uint8_t RUD_CH = 3;
constexpr uint8_t MAN_CH = 4;
constexpr uint8_t MOD_CH = 5;
constexpr uint8_t NUM_CH = 6;

class Rc_handler : public Module
{
public:
	Rc_handler(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<Modes_data> _modes_sub;

	Publisher<RC_data> _rc_pub;

	Modes_data _modes_data;
	RC_data _rc_data;
};

#endif /* MODULES_RC_HANDLER_RC_HANDLER_H_ */
