#ifndef LIB_DATA_BUS_MODES_H_
#define LIB_DATA_BUS_MODES_H_

enum class System_mode
{
	LOAD_PARAMS,
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
	MISSION
};

enum class Manual_mode
{
	DIRECT,
	STABILIZED
};

#endif /* LIB_DATA_BUS_MODES_H_ */
