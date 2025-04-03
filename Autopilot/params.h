#ifndef PARAMS_H_
#define PARAMS_H_

#include "lib/parameters/parameters.h"

// Generate all parameter declarations using the X-Macro pattern
#define PARAM(name, type) extern const param_t name;
#include "params_def.h"
#undef PARAM

// Initialize all parameters
void init_params(void);

#endif /* PARAMS_H_ */
