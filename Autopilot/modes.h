#ifndef MODES_H_
#define MODES_H_

enum class System_mode
{
	CONFIG,
	STARTUP,
	FLIGHT
};

enum class Flight_mode
{
	AUTO,
	MANUAL
};

enum class Auto_mode
{
	TAKEOFF,
	MISSION,
	LAND,
	FLARE,
	TOUCHDOWN
};

enum class Manual_mode
{
	DIRECT,
	STABILIZED
};

#endif /* MODES_H_ */
