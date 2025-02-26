#ifndef MODULES_MIXER_MIXER_H_
#define MODULES_MIXER_MIXER_H_

#include "Lib/Utils/utils.h"
#include "hal.h"
#include "parameters.h"

class Control_allocator
{
public:
	Control_allocator(HAL* hal, Plane* plane);

	void update();
private:
	Plane* _plane;
	HAL* _hal;

	uint16_t _elevator_duty = 0;
	uint16_t _aileron_duty = 0;
	uint16_t _throttle_duty = 0;
};

#endif /* MODULES_MIXER_MIXER_H_ */
