/*
 * telem.h
 *
 *  Created on: Jan 28, 2025
 *      Author: jeffr
 */

#ifndef TELEM_H_
#define TELEM_H_

#include "hal.h"

struct telem_packet
{
	float alt;
};

class Telem
{
public:
	Telem(Plane* plane, HAL* hal);
	void transmit();
private:
	HAL* _hal;
	Plane* _plane;
};

#endif /* TELEM_H_ */
