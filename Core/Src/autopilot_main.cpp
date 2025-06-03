#include "Autopilot_HAL/Autopilot_HAL.h"
#include "autopilot_main.h"
#include "autopilot.h"

void autopilot_main_c()
{
	AutopilotHAL hal; // Maybe declare this outside the function because after function completes, it gets destroyed and when interrupt calls it hardfault because its already destroyed
	Autopilot autopilot(&hal);
	autopilot.setup();

	// Bug: Hardfault handler when this removed
	while (1);
}
