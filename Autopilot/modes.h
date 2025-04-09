#ifndef MODES_H_
#define MODES_H_

enum class System_mode
{
	LOAD_PARAMS,
	STARTUP,
	FLIGHT,
	CALIBRATION
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
	FLARE
};

enum class Manual_mode
{
	DIRECT,
	STABILIZED
};

#endif /* MODES_H_ */
