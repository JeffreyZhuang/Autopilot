#include <autopilot.h>

Autopilot::Autopilot(HAL * hal, Plane * plane): ahrs(plane, hal), navigation(hal, plane)
{
    _hal = hal;
    _plane = plane;
};

void Autopilot::init()
{
    ahrs.setup();
    _hal->init();
}

void Autopilot::main_task()
{
    _hal->read_sensors();
    _hal->write_sd();

    ahrs.update();
    navigation.update();
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
