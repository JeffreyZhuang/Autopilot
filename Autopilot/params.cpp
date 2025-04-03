#include "params.h"

void init_params(void) {
    #define PARAM(name, type) name = param_add(#name, type);
    #include "params_def.h"
    #undef PARAM
}
