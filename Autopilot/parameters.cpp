#include "parameters.h"

// Singleton instance of Parameters
static Parameters instance;

const Parameters* get_params()
{
	return &instance;
}

void set_params(const Parameters* params)
{
	memcpy(&instance, params, sizeof(Parameters));
}
