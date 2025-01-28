#include <autopilot.h>

Autopilot* Autopilot::_instance = nullptr;

Autopilot::Autopilot(HAL* hal, Plane* plane): _ahrs(plane, hal, hal->main_dt),
									   	   	  _navigation(hal, plane, hal->main_dt),
											  _commander(hal, plane),
											  _control(hal, plane, hal->control_dt),
											  _guidance(hal, plane)
{
	_hal = hal;
	_plane = plane;
	_instance = this;
}

void Autopilot::init()
{
	_hal->init();
	_ahrs.setup();

	_hal->set_main_task(Autopilot::static_main_task);
	_hal->set_background_task(Autopilot::static_logger_task);
}

void Autopilot::main_task()
{
//	printf("%d\n", _plane->rc_switch);

	_plane->time = _hal->get_time_us();

	_hal->read_sensors();
	_ahrs.update();
	_navigation.execute();

	if (_plane->time - prev_control_time >= _hal->control_dt * 1000000)
	{
		_guidance.update();
		_control.update();
	}

	_hal->write_storage_buffer();
	_commander.update();

	char tx_buff[] = "Hello testing";
	_hal->transmit_telem((uint8_t*)tx_buff, strlen(tx_buff));

	char txBuf[200];
//	sprintf(txBuf,
//			"%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
//			_plane->ahrs_roll,
//			_plane->ahrs_pitch,
//			_plane->ahrs_yaw,
//			_plane->nav_acc_north,
//			_plane->nav_acc_east,
//			_plane->nav_acc_down,
//			_plane->nav_vel_down,
//			_plane->nav_pos_down,
//			_plane->nav_pos_east);
	sprintf(txBuf,
			"%.0f\t%.0f\t%.0f\t%.1f\t%.1f\t%.1f\t%.0f\t%.0f\t%.0f\r\n",
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

	_plane->loop_execution_time = _hal->get_time_us() - _plane->time;
//	printf("%ld ", (uint32_t)_plane->loop_execution_time);

	_plane->loop_iteration++;
}

void Autopilot::logger_task()
{
	switch (_plane->flightState)
	{
	case FlightState::STARTUP:
		break;
	case FlightState::TAKEOFF_DETECT:
		break;
	case FlightState::TAKEOFF:
		_hal->flush_storage_buffer();
		break;
	case FlightState::CRUISE:
		break;
	case FlightState::LAND:
//		_hal->read_storage();
		break;
	case FlightState::STABALIZE:
		break;
	case FlightState::MANUAL:
		break;

	}
}
