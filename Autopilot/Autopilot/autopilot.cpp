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
	sprintf(txBuf,
			"%f,%f,%f,%f,%f,%f,%f,%f\r\n",
			_plane->ahrs_roll,
			_plane->ahrs_pitch,
			_plane->ahrs_yaw,
			_plane->nav_acc_north,
			_plane->nav_acc_east,
			_plane->nav_acc_down,
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
