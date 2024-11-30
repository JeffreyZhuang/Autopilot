#include <hal.h>

HAL::HAL(Plane * plane): spi_bus(PB5, PB4, PA5), imu(spi_bus, PC15), baro(PC14, &spi_bus, 20000000, OSR_ULTRA_HIGH), 
                                         i2c_bus(PB9, PB8), ina219(&i2c_bus, 0x40, 0.01), swo(2000000) {
    _plane = plane;
}

void HAL::delay_us(uint32_t us) {
    delayMicroseconds(us);
}

void HAL::swo_print(char * str) {
    swo.print(str);
}

void HAL::usb_print(char * str) {
    Serial.print(str);
}

uint32_t HAL::get_time_us() {
    return micros();
}

void HAL::blink_led() {
    digitalWrite(PC1, HIGH);
    delay_us(500000);
    digitalWrite(PC1, LOW);
    delay_us(500000);
}

void HAL::setup() {
    setup_sd(); // BUG: Datalog setup does not work when moved under i2c_begin. I haven't defined the SDIO pins, maybe pins override i2c?
    setup_peripherals();
    setup_sensors();
}

void HAL::setup_peripherals() {
    Serial.begin(115200);
    pinMode(PC1, OUTPUT);

    i2c_bus.begin();
    i2c_bus.setClock(100000);
}

void HAL::setup_sensors() {
    imu.begin();
    baro.begin();
    mag.begin_SPI(PC13, &spi_bus);
}

void HAL::setup_sd() {
    swo.println("Initizliaing SD card...");
    while (!SD.begin(SD_DETECT_NONE)) {
        delay(10);
    }
    swo.println("Initialization done");

    file = SD.open("datalog.txt", FILE_WRITE);
    if (!file) {
        swo.println("error opening test.txt");
    }
}

void HAL::write_sd() {
    file.println(String(micros()) + "," + String(_plane->baro_alt) + "," + String(_plane->imu_az));
    file.flush();
}

void HAL::poll() {
    poll_imu();
    poll_compass();
    poll_barometer();
    poll_power_monitor();
}

void HAL::poll_imu() {
    imu.getAGT();
    _plane->imu_ax = imu.accX();
    _plane->imu_ay = imu.accY();
    _plane->imu_az = imu.accZ();
    _plane->imu_gx = imu.gyrX();
    _plane->imu_gy = imu.gyrY();
    _plane->imu_gz = imu.gyrZ();
    _plane->imu_temp = imu.temp();
    // Insert code here to rotate IMU data into correct frame, maybe imu_correction() function
}

void HAL::poll_compass() {
    mag.readDataNonBlocking(&_plane->compass_mx, &_plane->compass_my, &_plane->compass_mz);
}

void HAL::poll_barometer() {
    if (baro.read()) {
        _plane->baro_alt = (pow(1013.25/baro.getPressure(), 1.0 / 5.257) - 1.0) * (baro.getTemperature() + 273.15) / 0.0065;
    }
}

void HAL::poll_power_monitor() {
    _plane->batt_voltage = analogRead(PC0) * (3.3 / 1023.0);
    _plane->batt_current = analogRead(PC2) * (3.3 / 1023.0);
    _plane->autopilot_voltage = ina219.read_voltage();
    _plane->autopilot_current = ina219.read_current();
}