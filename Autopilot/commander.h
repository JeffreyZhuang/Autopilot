#ifndef COMMANDER_H_
#define COMMANDER_H_

#include "hal.h"
#include "plane.h"

/*
 * Detects and executes state transitions
 */

class Commander
{
public:
	Commander(HAL* hal, Plane* plane);
	void update();
private:
	HAL* _hal;
	Plane* _plane;
};

#endif /* COMMANDER_H_ */
