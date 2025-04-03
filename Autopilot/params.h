#ifndef PARAMS_H_
#define PARAMS_H_

#include "lib/parameters/parameters.h"

// Declare all parameter handles
#define PARAM(name, type) extern param_t name;
#include "params_def.h"
#undef PARAM

// Function to initialize all parameters
void create_params(void);

#endif /* PARAMS_H_ */
