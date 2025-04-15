#include "params.h"

// Instantiate parameters
#define PARAM(name, type, default) type name(#name, default);
#include "params_def.h"
#undef PARAM

void params_init(void) {
    // Register all parameters
    #define PARAM(name, type, default) param_register(&name);
    #include "params_def.h"
    #undef PARAM
}
