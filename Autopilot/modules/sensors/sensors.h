#ifndef MODULES_SENSORS_SENSORS_H_
#define MODULES_SENSORS_SENSORS_H_

#include "parameters.h"
#include "module.h"

struct Hitl_rx_packet
{
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
	float mx;
	float my;
	float mz;
	float asl;
	int32_t lat;
	int32_t lon;
	int16_t of_x;
	int16_t of_y;
};

class Sensors : Module
{
public:
	Sensors(HAL* hal, Data_bus* data_bus);

	void update() override;

private:
	Subscriber<Modes_data> _modes_sub;

	Publisher<IMU_data> _imu_pub;
	Publisher<Mag_data> _mag_pub;
	Publisher<Baro_data> _baro_pub;
	Publisher<OF_data> _of_pub;
	Publisher<GNSS_data> _gnss_pub;
	Publisher<Power_data> _power_pub;

	Modes_data _modes_data;

	void read_sensors();
	void read_hitl();
};

#endif /* MODULES_SENSORS_SENSORS_H_ */
