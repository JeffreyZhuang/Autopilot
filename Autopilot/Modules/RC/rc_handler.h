#ifndef MODULES_RC_RC_HANDLER_H_
#define MODULES_RC_RC_HANDLER_H_

#include "Lib/Utils/utils.h"
#include "parameters.h"
#include "hal.h"
#include "plane.h"
#include <stdio.h>

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
