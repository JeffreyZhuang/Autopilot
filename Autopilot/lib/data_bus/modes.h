#ifndef LIB_DATA_BUS_MODES_H_
#define LIB_DATA_BUS_MODES_H_

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
	WAYPOINT,
	LOITER,
	HOLD, // Circular holding pattern for landing
	LAND,
	FLARE
};

enum class Manual_mode
{
	DIRECT,
	STABILIZED
};

#endif /* LIB_DATA_BUS_MODES_H_ */
