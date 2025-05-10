#ifndef LIB_PARAMETERS_PARAMS_H_
#define LIB_PARAMETERS_PARAMS_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    PARAM_TYPE_INT32,
    PARAM_TYPE_FLOAT,
    PARAM_TYPE_UNKNOWN
} param_type_t;

typedef uint16_t param_t;

#define PARAM_INVALID 0xFFFF

// Declare all parameter handles
#define PARAM(name, type) extern param_t name;
#include "params_def.h"
#undef PARAM

void param_init(void);
param_t param_find(const char *name);
int param_get(param_t param, void *val);
int param_set_int32(param_t param, int32_t val);
int param_set_float(param_t param, float val);
param_type_t param_get_type(param_t param);
bool param_all_set(void);

#endif /* LIB_PARAMETERS_PARAMS_H_ */
