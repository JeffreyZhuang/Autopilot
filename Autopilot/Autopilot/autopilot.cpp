#include <autopilot.h>

Autopilot* Autopilot::_instance = nullptr;

Autopilot::Autopilot(HAL* hal, Plane* plane): _ahrs(plane, hal),
									   	   	  _navigation(hal, plane),
											  _commander(hal, plane),
											  _control(hal, plane),
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
	_plane->time = _hal->get_time_us();

	_hal->read_sensors();
	_ahrs.update();
	_navigation.execute();

	_hal->xitl_run();

	// 50hz
	if (_plane->loop_iteration % 2 == 0)
	{
		_guidance.update();
		_control.update();
	}

	_hal->write_storage_buffer();
	_commander.update();

	// Send struct instead and then decode struct in Python
	char txBuf[200];
	sprintf(txBuf,
			"%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",
			_plane->ahrs_roll,
			_plane->ahrs_pitch,
			_plane->ahrs_yaw,
			_plane->nav_acc_north,
			_plane->nav_acc_east,
			_plane->nav_acc_down,
			_plane->nav_vel_down,
			_plane->nav_pos_down,
			_plane->nav_pos_east);
	_hal->usb_print(txBuf);

	_plane->loop_execution_time = _hal->get_time_us() - _plane->time;
//	printf("%ld ", (uint32_t)_plane->loop_execution_time);

	_plane->loop_iteration++;
}

void Autopilot::logger_task()
{
	switch (_plane->flightState)
	{
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
