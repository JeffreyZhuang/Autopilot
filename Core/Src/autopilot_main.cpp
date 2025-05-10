#include "Autopilot_HAL/Autopilot_HAL.h"
#include "autopilot_main.h"
#include "autopilot.h"

void autopilot_main_c()
{
	AutopilotHAL hal;
	Autopilot autopilot(&hal);
	autopilot.setup();

	// Bug: Hardfault handler when this removed
	while (1);
}
