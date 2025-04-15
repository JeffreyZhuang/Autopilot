#include "parameters.h"

#define MAX_PARAMS 100

static ParamBase* param_table[MAX_PARAMS];
static uint16_t param_count = 0;

void param_register(ParamBase* param) {
    if (param_count < MAX_PARAMS) {
        param_table[param_count++] = param;
    }
}

ParamBase* param_find(const char* name) {
    uint32_t target_hash = 5381;
    const char* p = name;
    while (*p) {
        target_hash = ((target_hash << 5) + target_hash) + *p++;
    }

    for (uint16_t i = 0; i < param_count; i++) {
        if (param_table[i]->hash == target_hash) {
            return param_table[i];
        }
    }
    return NULL;
}

bool param_all_set(void) {
    for (uint16_t i = 0; i < param_count; i++) {
        if (!param_table[i]->is_set) {
            return false;
        }
    }
    return true;
}
