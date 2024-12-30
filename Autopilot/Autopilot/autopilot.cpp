#include <autopilot.h>

Autopilot::Autopilot(HAL * hal, Plane * plane): _ahrs(plane, hal),
												_navigation(hal, plane),
												_commander(hal, plane)
{
	_hal = hal;
	_plane = plane;
};

void Autopilot::init()
{
	_ahrs.setup();
	_hal->init();
}

void Autopilot::main_task()
{
	_hal->read_sensors();
	_ahrs.update();
	_navigation.execute();
	_hal->write_storage_buffer();
	_commander.update();

	char txBuf[200];
	sprintf(txBuf,
			"%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\r\n",
			_plane->ahrs_roll,
			_plane->ahrs_pitch,
			_plane->ahrs_yaw,
			_plane->imu_gx,
			_plane->imu_gy,
			_plane->imu_gz,
			_plane->imu_ax,
			_plane->imu_ay,
			_plane->imu_az,
			_plane->compass_mx,
			_plane->compass_my,
			_plane->compass_mz);
	_hal->usb_print(txBuf);
}

void Autopilot::logger_task()
{
	switch (_plane->flightState)
	{
	case FlightState::TAKEOFF:
		_hal->flush_storage_buffer();
		break;
	case FlightState::LAND:
		_hal->read_storage();
		break;
	}
}
