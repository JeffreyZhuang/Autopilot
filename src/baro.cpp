#include <hal_arduino.h>

/**
 * @brief Setup barometer
 * 
 */
void HAL_Arduino::setup_barometer() {
    baro.begin();
}

/**
 * @brief Poll data from barometer and update plane struct
 * 
 */
void HAL_Arduino::poll_barometer() {
    if (baro.read()) {
        _plane->baro_alt = (pow(1013.25/baro.getPressure(), 1.0 / 5.257) - 1.0) * (baro.getTemperature() + 273.15) / 0.0065;
        _plane->baro_timestamp = get_time_us();
    }
}