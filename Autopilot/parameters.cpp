#include "parameters.h"

static Params_payload instance;
static bool params_initialized = false;

const Params_payload* get_params()
{
	return &instance;
}

void set_params(const Params_payload* params)
{
	memcpy(&instance, params, sizeof(Params_payload));
	params_initialized = true;
}

bool are_params_set()
{
	return params_initialized;
}
