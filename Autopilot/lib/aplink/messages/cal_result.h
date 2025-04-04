#ifndef LIB_APLINK_MESSAGES_CAL_RESULT_H_
#define LIB_APLINK_MESSAGES_CAL_RESULT_H_

struct aplink_cal_result
{
	float gyr_off_x;
	float gyr_off_y;
	float gyr_off_z;
	float acc_off_x;
	float acc_off_y;
	float acc_off_z;
	float hi_x;
	float hi_y;
	float hi_z;
	float si_xx;
	float si_xy;
	float si_xz;
	float si_yx;
	float si_yy;
	float si_yz;
	float si_zx;
	float si_zy;
	float si_zz;
};

#endif /* LIB_APLINK_MESSAGES_CAL_RESULT_H_ */
