#include "params.h"

// Define all parameters
#define PARAM(name, type) param_t name;
#include "params_def.h"
#undef PARAM

// Function to register parameters
void create_params(void) {
    param_init();

    #define PARAM(name, type) name = param_add(#name, type);
    #include "params_def.h"
    #undef PARAM
}
