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
void Autopilot::setup()
{
    ahrs.setup();
    _hal->setup();
}

/**
 * @brief Call every main loop iteration to update Autopilot
 *
 */
void Autopilot::loop()
{
    _hal->poll();

    uint32_t dt = _hal->get_time_us() - prev_loop_time;
    prev_loop_time = _hal->get_time_us();

    ahrs.update();
    navigation.update();
}

void Autopilot::logging_loop()
{
	_hal->write_sd();
}
