#include "Autopilot_HAL/Autopilot_HAL.h"

void AutopilotHAL::init_imu()
{
	_imu.begin();

	_imu.setAccelODR(ICM42688::odr100);
	_imu.setAccelFS(ICM42688::gpm4);

	_imu.setGyroODR(ICM42688::odr100);
	_imu.setGyroFS(ICM42688::dps500);
}

bool AutopilotHAL::read_imu(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
	if (_imu.getAGT() == 1)
	{
		*ax = -_imu.accX();
		*ay = -_imu.accY();
		*az = _imu.accZ();
		*gx = -_imu.gyrX();
		*gy = -_imu.gyrY();
		*gz = _imu.gyrZ();

		return true;
	}

	return false;
}
