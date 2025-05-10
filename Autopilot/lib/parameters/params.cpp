#include <lib/parameters/params.h>
#include <string.h>
#include <stdbool.h>

typedef struct {
    const char *name;
    param_type_t type;
    bool is_set;           // Flag indicating if parameter has been set
    union {
        int32_t i32_val;   // Integer value
        float float_val;   // Float value
    } value;
} param_entry_t;

#define MAX_PARAMS 256
static param_entry_t param_table[MAX_PARAMS];
static uint16_t param_count = 0;

static uint32_t param_hash(const char *str) {
    uint32_t hash = 5381;
    int c;
    while ((c = *str++)) {
        hash = ((hash << 5) + hash) + c; // hash * 33 + c
    }
    return hash;
}

void param_init(void) {
    memset(param_table, 0, sizeof(param_table));
    param_count = 0;
}

param_t param_add(const char *name, param_type_t type) {
    if (param_count >= MAX_PARAMS) return PARAM_INVALID;

    uint32_t hash = param_hash(name);
    param_table[param_count].name = name;
    param_table[param_count].type = type;
    param_table[param_count].is_set = false;

    return param_count++; // Return handle (index)
}

param_t param_find(const char *name) {
    uint32_t target_hash = param_hash(name);

    for (param_t i = 0; i < param_count; i++) {
        if (param_hash(param_table[i].name) == target_hash) {
            return i; // Return handle (index)
        }
    }

    return PARAM_INVALID; // Not found
}

int param_get(param_t param, void *val) {
    if (param >= param_count || !param_table[param].is_set) {
        return -1; // Error: parameter not valid or not set
    }

    switch (param_table[param].type) {
        case PARAM_TYPE_INT32:
            *((int32_t *)val) = param_table[param].value.i32_val;
            return 0;
        case PARAM_TYPE_FLOAT:
            *((float *)val) = param_table[param].value.float_val;
            return 0;
        default:
            return -1; // Error: unknown type
    }
}

int param_set_int32(param_t param, int32_t val) {
    if (param >= param_count || param_table[param].type != PARAM_TYPE_INT32) {
        return -1; // Error
    }
    param_table[param].value.i32_val = val;
    param_table[param].is_set = true;
    return 0;
}

int param_set_float(param_t param, float val) {
    if (param >= param_count || param_table[param].type != PARAM_TYPE_FLOAT) {
        return -1; // Error
    }
    param_table[param].value.float_val = val;
    param_table[param].is_set = true;
    return 0;
}

param_type_t param_get_type(param_t param) {
    if (param >= param_count) {
        return PARAM_TYPE_UNKNOWN;
    }
    return param_table[param].type;
}

// Check if all parameters have been set
bool param_all_set(void) {
    for (param_t i = 0; i < param_count; i++) {
        if (!param_table[i].is_set) {
            return false;
        }
    }
    return true;
}

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
