#ifndef PARAMS_H_
#define PARAMS_H_

#include "lib/parameters/parameters.h"

// Forward declarations
#define PARAM(name, type, default) extern type name;
#include "params_def.h"
#undef PARAM

void params_init(void);

#endif /* PARAMS_H_ */
