/*
 * pitl_hal.h
 *
 *  Created on: Dec. 30, 2024
 *      Author: jeffr
 */

#ifndef INC_PITL_HAL_H_
#define INC_PITL_HAL_H_

#include "hal.h"

// Processor in the loop using USB
class Pitl_hal : public HAL
{
public:
	Pitl_hal();
};


#endif /* INC_PITL_HAL_H_ */
