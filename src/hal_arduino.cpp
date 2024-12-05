#include <hal_arduino.h>

/**
 * @brief Construct a new hal arduino::hal arduino object
 * 
 * @param plane 
 */
HAL_Arduino::HAL_Arduino(Plane * plane): HAL(plane), 
                                         spi_bus(PB5, PB4, PA5), 
                                         imu(spi_bus, PC15), 
                                         baro(PC14, &spi_bus, 20000000, OSR_ULTRA_HIGH), 
                                         i2c_bus(PB9, PB8), 
                                         ina219(&i2c_bus, 0x40, 0.01), 
                                         swo(2000000) {
    _plane = plane;
}

/**
 * @brief Setup all of the hardware
 * 
 */
void HAL_Arduino::setup() {
    setup_sd(); // BUG: Datalog setup does not work when moved under i2c_begin. I haven't defined the SDIO pins, maybe pins override i2c?
    setup_peripherals();
    setup_sensors();
}

/**
 * @brief Pause processor by microseconds
 * 
 * @param us 
 */
void HAL_Arduino::delay_us(uint32_t us) {
    delayMicroseconds(us);
}

/**
 * @brief Debug print using serial wire output trace
 * 
 * @param str 
 */
void HAL_Arduino::swo_print(char * str) {
    swo.print(str);
}

/**
 * @brief Print message over USB CDC
 * 
 * @param str 
 */
void HAL_Arduino::usb_print(char * str) {
    Serial.print(str);
}

/**
 * @brief Get the time in microseconds
 * 
 * @return uint32_t 
 */
uint32_t HAL_Arduino::get_time_us() {
    return micros();
}

/**
 * @brief Toggle the LED
 * 
 */
void HAL_Arduino::toggle_led() {
    digitalWrite(PC1, !digitalRead(PC1));
}

/**
 * @brief Setup hardware peripherals
 * 
 */
void HAL_Arduino::setup_peripherals() {
    Serial.begin(115200);

    pinMode(PC1, OUTPUT);

    i2c_bus.begin();
    i2c_bus.setClock(100000);
}

/**
 * @brief Setup sensors
 * 
 */
void HAL_Arduino::setup_sensors() {
    setup_imu();
    setup_barometer();
    setup_compass();
}

/**
 * @brief Poll data from sensors
 * 
 */
void HAL_Arduino::poll() {
    poll_imu();
    poll_compass();
    poll_barometer();
    poll_power_monitor();
}

/**
 * @brief Scan I2C bus
 * 
 */
void HAL_Arduino::i2c_scan() {

}