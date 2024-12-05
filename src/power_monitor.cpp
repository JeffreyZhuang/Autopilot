#include <hal_arduino.h>

/**
 * @brief Poll power monitor data and update plane struct
 * 
 */
void HAL_Arduino::poll_power_monitor() {
    _plane->batt_voltage = analogRead(PC0) * (3.3 / 1023.0);
    _plane->batt_current = analogRead(PC2) * (3.3 / 1023.0);
    _plane->autopilot_voltage = ina219.read_voltage();
    _plane->autopilot_current = ina219.read_current();
}