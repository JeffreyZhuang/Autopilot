#ifndef LIB_PARAMETERS_PARAMETERS_H_
#define LIB_PARAMETERS_PARAMETERS_H_

#include <stdint.h>
#include <stdbool.h>

// Parameter types
typedef enum {
    PARAM_TYPE_INT32,
    PARAM_TYPE_FLOAT,
    PARAM_TYPE_UNKNOWN
} param_type_t;

// Parameter handle (acts as an index)
typedef uint16_t param_t;

// Invalid handle
#define PARAM_INVALID 0xFFFF

// Initialize the parameter system
void param_init(void);

// Add parameter
param_t param_add(const char *name, param_type_t type);

// Find a parameter by name (returns handle)
param_t param_find(const char *name);

// Get/set parameter values
int param_get_int32(param_t param, int32_t *val);
int param_get_float(param_t param, float *val);
int param_set_int32(param_t param, int32_t val);
int param_set_float(param_t param, float val);

#endif /* LIB_PARAMETERS_PARAMETERS_H_ */
