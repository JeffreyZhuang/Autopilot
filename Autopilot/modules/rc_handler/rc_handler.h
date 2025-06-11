#ifndef MODULES_RC_HANDLER_RC_HANDLER_H_
#define MODULES_RC_HANDLER_RC_HANDLER_H_

#include <lib/hal/hal.h>
#include <lib/module/module.h>
#include "lib/parameters/params.h"
#include "lib/data_bus/data_bus.h"
#include "lib/utils/utils.h"
#include <stdio.h>

// Channels
constexpr uint8_t AIL_CH = 0;
constexpr uint8_t ELE_CH = 1;
constexpr uint8_t THR_CH = 2;
constexpr uint8_t RUD_CH = 3;
constexpr uint8_t MAN_CH = 4;
constexpr uint8_t MOD_CH = 5;
constexpr uint8_t NUM_CH = 6;

constexpr uint16_t TX_CONN_THRESHOLD = 500; // Transmitter channels must be above this PWM value

class RCHandler : public Module
{
public:
	RCHandler(HAL* hal, DataBus* data_bus);

	void update() override;

private:
	Subscriber<Modes_data> _modes_sub;

	Publisher<RC_data> _rc_pub;

	Modes_data _modes_data;
	RC_data _rc_data;

	// Parameters
	int32_t _min_duty;
	int32_t _max_duty;

	void parameters_update();
};

#endif /* MODULES_RC_HANDLER_RC_HANDLER_H_ */
