#ifndef MODULES_RCHANDLER_RC_HANDLER_H_
#define MODULES_RCHANDLER_RC_HANDLER_H_

#include "Lib/Utils/utils.h"
#include "parameters.h"
#include "hal.h"
#include "plane.h"
#include "module.h"
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
	Rc_handler(HAL* hal, Plane* plane);

	void update();
};

#endif /* MODULES_RCHANDLER_RC_HANDLER_H_ */
