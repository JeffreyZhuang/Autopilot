/*
 * tecs.h
 *
 *  Created on: Feb. 5, 2025
 *      Author: jeffr
 */

#ifndef LIB_TECS_TECS_H_
#define LIB_TECS_TECS_H_

#include "Lib/Utils/utils.h"
#include "plane.h"
#include <math.h>

class Tecs
{
public:
	Tecs(Plane* plane);
	void update();
private:
	Plane* _plane;
};


#endif /* LIB_TECS_TECS_H_ */
