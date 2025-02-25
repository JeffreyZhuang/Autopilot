#ifndef MODULES_CONTROLALLOCATOR_CONTROL_ALLOCATOR_H_
#define MODULES_CONTROLALLOCATOR_CONTROL_ALLOCATOR_H_

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

#endif /* MODULES_CONTROLALLOCATOR_CONTROL_ALLOCATOR_H_ */
