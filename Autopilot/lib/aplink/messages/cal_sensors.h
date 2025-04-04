#ifndef LIB_APLINK_MESSAGES_CAL_SENSORS_H_
#define LIB_APLINK_MESSAGES_CAL_SENSORS_H_

static constexpr uint8_t CAL_SENSORS_MSG_ID = 15;

struct aplink_cal_sensors
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
};

DECLARE_APLINK_MESSAGE(aplink_cal_sensors, CAL_SENSORS_MSG_ID);

#endif /* LIB_APLINK_MESSAGES_CAL_SENSORS_H_ */
