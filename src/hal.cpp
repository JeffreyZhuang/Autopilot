#include <hal.h>

// Add print and delay here later on

HAL::HAL(Plane * plane): spi_bus(PB5, PB4, PA5), imu(spi_bus, PC15), baro(PC14, &spi_bus, 20000000, OSR_ULTRA_HIGH), 
                                     i2c_bus(PB9, PB8), ina219(&i2c_bus, 0x40, 0.01) {
    _plane = plane;
}

void HAL::setup() {
    setup_peripherals();
    setup_sensors();
}

void HAL::setup_peripherals() {
    i2c_bus.begin();
    i2c_bus.setClock(100000);
}

void HAL::setup_sensors() {
    imu.begin();
    baro.begin();
    mag.begin_SPI(PC13, &spi_bus);
}

void HAL::setup_sd() {

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