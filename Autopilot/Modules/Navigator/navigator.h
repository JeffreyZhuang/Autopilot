#ifndef MODULES_NAVIGATOR_NAVIGATOR_H_
#define MODULES_NAVIGATOR_NAVIGATOR_H_

#include "Lib/Utils/utils.h"
#include "plane.h"
#include "hal.h"
#include "parameters.h"
#include "module.h"

class Navigator : public Module
{
public:
	Navigator(HAL* hal, Plane* plane);

	void update() override;
};


#endif /* MODULES_NAVIGATOR_NAVIGATOR_H_ */
