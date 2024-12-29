#include <autopilot.h>

/**
 * @brief Construct a new Autopilot:: Autopilot object
 *
 * @param hal
 * @param plane
 */
Autopilot::Autopilot(HAL * hal, Plane * plane): ahrs(plane, hal), navigation(hal, plane)
{
    _hal = hal;
    _plane = plane;
};

/**
 * @brief Call once in the beginning to setup Autopilot
 *
 */
void Autopilot::init()
{
    ahrs.setup();
    _hal->init();
}

/**
 * @brief Call every main loop iteration to update Autopilot
 *
 */
void Autopilot::main_task()
{
    _hal->read_sensors();
    _hal->write_sd();

    uint32_t dt = _hal->get_time_us() - prev_loop_time;
    prev_loop_time = _hal->get_time_us();

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
