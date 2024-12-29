#include <autopilot.h>

Autopilot::Autopilot(HAL * hal, Plane * plane): _ahrs(plane, hal), _navigation(hal, plane)
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
    _hal->write_sd();
}

void Autopilot::logger_task()
{
	if (_hal->get_time_us() < 10000000)
	{
		_hal->flush_sd();
	}
	else
	{
		_hal->read_sd();
	}
}
