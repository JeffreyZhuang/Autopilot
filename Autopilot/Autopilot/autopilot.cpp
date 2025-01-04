#include <autopilot.h>

Autopilot* Autopilot::_instance = nullptr;

void Autopilot::init()
{
	_hal->init();
	_ahrs.setup();

	_hal->set_main_task(Autopilot::static_main_task);
	_hal->set_background_task(Autopilot::static_logger_task);
}

void Autopilot::main_task()
{
	_hal->read_sensors();
	_ahrs.update();
	_navigation.execute();
	_control.update();
	_hal->write_storage_buffer();
	_commander.update();

	char txBuf[200];
//	sprintf(txBuf,
//			"%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%.1f\t%f\t%f\r\n",
//			_plane->ahrs_roll,
//			_plane->ahrs_pitch,
//			_plane->ahrs_yaw,
//			_plane->imu_gx,
//			_plane->imu_gy,
//			_plane->imu_gz,
//			_plane->imu_ax,
//			_plane->imu_ay,
//			_plane->imu_az,
//			_plane->compass_mx,
//			_plane->compass_my,
//			_plane->compass_mz,
//			_plane->gnss_lat,
//			_plane->gnss_lon);
	sprintf(txBuf,
			"%f,%f,%f,%f,%f,%f,%f,%f\r\n",
			_plane->ahrs_roll,
			_plane->ahrs_pitch,
			_plane->ahrs_yaw,
			_navigation.acc_n,
			_navigation.acc_e,
			_navigation.acc_d,
			_plane->nav_vel_down,
			_plane->nav_pos_down);
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
//		_hal->read_storage();
		break;
	}
}
