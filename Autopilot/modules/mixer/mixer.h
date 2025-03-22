#ifndef MODULES_MIXER_MIXER_H_
#define MODULES_MIXER_MIXER_H_

#include "lib/utils/utils.h"
#include "hal.h"
#include "module.h"
#include "parameters.h"

class Mixer : public Module
{
public:
	Mixer(HAL* hal, Plane* plane);

	void update();

private:
	uint16_t _elevator_duty = 0;
	uint16_t _rudder_duty = 0;
	uint16_t _throttle_duty = 0;

	void update_config();
	void update_startup();
	void update_flight();
};

#endif /* MODULES_MIXER_MIXER_H_ */
