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
 * @brief Setup IMU
 * 
 */
void HAL_Arduino::setup_imu() {
    imu.setAccelODR(ICM42688::odr200);
    imu.setAccelFS(ICM42688::gpm4);

    imu.setGyroODR(ICM42688::odr200);
    imu.setGyroFS(ICM42688::dps500);

    imu.begin();
}

/**
 * @brief Setup compass
 * 
 */
void HAL_Arduino::setup_compass() {
    mag.begin_SPI(PC13, &spi_bus);
    mag.setGain(MLX90393_GAIN_1X);
    mag.setResolution(MLX90393_X, MLX90393_RES_17);
    mag.setResolution(MLX90393_Y, MLX90393_RES_17);
    mag.setResolution(MLX90393_Z, MLX90393_RES_16);
    mag.setOversampling(MLX90393_OSR_3);
    mag.setFilter(MLX90393_FILTER_5);
}

/**
 * @brief Setup barometer
 * 
 */
void HAL_Arduino::setup_barometer() {
    baro.begin();
}

/**
 * @brief Setup micro SD card
 * 
 */
void HAL_Arduino::setup_sd() {
    // Micro SD pin configuration
    SD.setDx(PC8, PC9, PC10, PC11);
    SD.setCMD(PD2);
    SD.setCK(PC12);

    // Initialization
    swo.print("Initizliaing SD card...\n");
    while (!SD.begin(SD_DETECT_NONE)) {
        delay(10);
    }
    swo.print("Initialization done\n");

    file = SD.open("datalog.txt", FILE_WRITE);
    if (!file) {
        swo.print("error opening test.txt\n");
    }
}

/**
 * @brief Write to micro SD card
 * 
 */
void HAL_Arduino::write_sd() {
    file.println(String(millis()) + "," + String(_plane->baro_alt, 2) + "," + String(_plane->imu_az, 1));
    swo.println("Before flush: " + String(micros()));
    file.flush();
    swo.println("After flush: " + String(micros()));
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
 * @brief Poll data from IMU
 * 
 */
void HAL_Arduino::poll_imu() {
    if (imu.getAGT()) {
        // Rotate IMU to correct coordinate system
        _plane->imu_ax = -imu.accX();
        _plane->imu_ay = imu.accY();
        _plane->imu_az = imu.accZ();
        _plane->imu_gx = -imu.gyrX();
        _plane->imu_gy = -imu.gyrY();
        _plane->imu_gz = imu.gyrZ();
        _plane->imu_temp = imu.temp();
        _plane->imu_timestamp = get_time_us();
    };
}

/**
 * @brief Poll data from compass and update plane struct
 * 
 */
void HAL_Arduino::poll_compass() {
    if (mag.readDataNonBlocking(&_plane->compass_mx, &_plane->compass_my, &_plane->compass_mz)) {
        _plane->compass_timestamp = get_time_us();
    }
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

/**
 * @brief Scan I2C bus
 * 
 */
void HAL_Arduino::i2c_scan() {
    return;
}