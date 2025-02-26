#ifndef MODULES_RCHANDLER_RC_HANDLER_H_
#define MODULES_RCHANDLER_RC_HANDLER_H_

#include "Lib/Utils/utils.h"
#include "parameters.h"
#include "hal.h"
#include "plane.h"

class Rc_handler
{
public:
	Rc_handler(HAL* hal, Plane* plane);
	void rc_update();
private:
	HAL* _hal;
	Plane* _plane;
};

#endif /* MODULES_RCHANDLER_RC_HANDLER_H_ */
