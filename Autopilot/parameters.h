#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "lib/autopilot_link/autopilot_link.h"
#include <stdint.h>
#include <cstring>

const Params_payload* get_params();
void set_params(const Params_payload* params);
bool are_params_set();

#endif /* PARAMETERS_H_ */
