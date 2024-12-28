#include <autopilot.h>

/**
 * @brief Construct a new Autopilot:: Autopilot object
 *
 * @param hal
 * @param plane
 */
Autopilot::Autopilot(HAL * hal, Plane * plane): ahrs(plane, hal), navigation(hal, plane) {
    _hal = hal;
    _plane = plane;
};

/**
 * @brief Call once in the beginning to setup Autopilot
 *
 */
void Autopilot::setup() {
    ahrs.setup();
    _hal->setup();
}

/**
 * @brief Call every main loop iteration to update Autopilot
 *
 */
void Autopilot::loop() {
    _hal->poll();

    uint32_t dt = _hal->get_time_us() - prev_loop_time;
    prev_loop_time = _hal->get_time_us();

    ahrs.update();
    navigation.update();
    // _hal->write_sd(); // This takes 4ms, and is also the cause of freezing/inconsistent dt. Without it, dt is very consistent.

    if (_hal->get_time_us() - prev_print_time > 100000) {
        float heading = (atan2(_plane->compass_my, _plane->compass_mx) * 180) / M_PI;
        if (heading < 0) {
           heading += 360;
        }

        strcpy(txBuf, "");
        sprintf(txBuf,
                "%d\t|\t%.2f\t|\t%.2f\t%.2f\t%.2f\t|\t%.1f\t%.1f\t%.1f\t|\t%.2f\t%.2f\t%.2f\t|\t%.1f\t%.1f\t%.1f\t%.1f\n",
                dt,
                _plane->baro_alt,
                _plane->imu_ax,
                _plane->imu_ay,
                _plane->imu_az,
                _plane->imu_gx,
                _plane->imu_gy,
                _plane->imu_gz,
                _plane->compass_mx,
                _plane->compass_my,
                _plane->compass_mz,
                _plane->ahrs_roll,
                _plane->ahrs_pitch,
                _plane->ahrs_yaw,
                heading);
        _hal->usb_print(txBuf);

        // _hal->toggle_led();

        prev_print_time = _hal->get_time_us();
    }
}
