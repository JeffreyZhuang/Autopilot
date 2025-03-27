#ifndef MODULES_NAVIGATOR_NAVIGATOR_H_
#define MODULES_NAVIGATOR_NAVIGATOR_H_

#include <data_bus.h>
#include "lib/utils/utils.h"
#include "hal.h"
#include "parameters.h"
#include "module.h"

class Navigator : public Module
{
public:
	Navigator(HAL* hal, Plane* plane);

	void update() override;

private:
	Plane::Subscription_handle pos_est_handle;
};


#endif /* MODULES_NAVIGATOR_NAVIGATOR_H_ */
