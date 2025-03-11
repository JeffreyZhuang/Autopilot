#include "parameters.h"

static Parameters instance;
static bool params_initialized = false;

// Add default parameters incase you access params before it has been initialized

const Parameters* get_params()
{
	return &instance;
}

void set_params(const Parameters* params)
{
	memcpy(&instance, params, sizeof(Parameters));
	params_initialized = true;
}

bool are_params_set()
{
	return params_initialized;
}
