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
	_navigation.update();
	_hal->write_storage_buffer();
	_commander.update();

	char txBuf[200];
	sprintf(txBuf,
			"%.1f\t%.1f\t%.1f\r\n",
			_plane->ahrs_roll,
			_plane->ahrs_pitch,
			_plane->ahrs_yaw);
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
