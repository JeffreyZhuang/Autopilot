#ifndef MODULES_RC_RC_HANDLER_H_
#define MODULES_RC_RC_HANDLER_H_

#include "Lib/Utils/utils.h"
#include "parameters.h"
#include "hal.h"
#include "plane.h"
#include <stdio.h>

constexpr uint16_t TX_DETECT_MIN_DUTY = 500; // Transmitter is detected if value higher than this

class Rc_handler
{
public:
	Rc_handler(HAL* hal, Plane* plane);
	void rc_update();
private:
	HAL* _hal;
	Plane* _plane;
};

#endif /* MODULES_RC_RC_HANDLER_H_ */
