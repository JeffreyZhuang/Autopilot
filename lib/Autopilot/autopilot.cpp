#include <autopilot.h>

// Autopilot::Autopilot(HAL * hal, Plane * plane): ahrs(plane, hal), navigation(hal, plane) {
Autopilot::Autopilot(HAL * hal, Plane * plane) {
    _hal = hal;
    _plane = plane;
};

void Autopilot::setup() {
    // ahrs.setup();
    _hal->setup();
}

void Autopilot::loop() {
    _hal->poll();

    uint32_t dt = _hal->get_time_us() - prev_loop_time;
    prev_loop_time = _hal->get_time_us();

    // ahrs.update();
    // navigation.update();
    _hal->write_sd();

    strcpy(txBuf, ""); 
    sprintf(txBuf, "DT: %d\n", dt);
    _hal->usb_print(txBuf);

    if (_hal->get_time_us() - prev_print_time > 300000) {
        strcpy(txBuf, ""); 
        sprintf(txBuf, "%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", _plane->baro_alt, 
                                                                     _plane->imu_ax, 
                                                                     _plane->imu_ay, 
                                                                     _plane->imu_gz, 
                                                                     _plane->compass_mx, 
                                                                     _plane->compass_my, 
                                                                     _plane->compass_mz);
        _hal->swo_print(txBuf);

        _hal->toggle_led();

        prev_print_time = _hal->get_time_us();
    }
}