#ifndef LIB_APLINK_MESSAGES_HITL_INPUT_H_
#define LIB_APLINK_MESSAGES_HITL_INPUT_H_

#include "lib/aplink/aplink_types.h"

static constexpr uint8_t HITL_INPUT_MSG_ID = 4;

struct __attribute__((packed)) aplink_hitl_input
{
	float imu_ax;
	float imu_ay;
	float imu_az;
	float imu_gx;
	float imu_gy;
	float imu_gz;
	float mag_x;
	float mag_y;
	float mag_z;
	float baro_asl;
	int32_t gps_lat;
	int32_t gps_lon;
	int16_t of_x;
	int16_t of_y;
};

DECLARE_APLINK_MESSAGE(aplink_hitl_input, HITL_INPUT_MSG_ID);

#endif /* LIB_APLINK_MESSAGES_HITL_INPUT_H_ */
