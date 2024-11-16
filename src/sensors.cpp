#include <sensors.h>

Sensors::Sensors(Vehicle * vehicle): spi_bus(PB5, PB4, PA5), imu(spi_bus, PC15), baro(PC14, &spi_bus, 20000000, OSR_ULTRA_HIGH), 
                                     i2c_bus(PB9, PB8), ina219(&i2c_bus, 0x40, 0.01) {
    _vehicle = vehicle;
}

void Sensors::setup() {
    // Setup peripherals
    i2c_bus.begin();
    i2c_bus.setClock(100000);

    // Setup sensors
    imu.begin();

    baro.begin();

    mag.begin_SPI(PC13, &spi_bus);
    mag.setGain(MLX90393_GAIN_1X);
    mag.setResolution(MLX90393_X, MLX90393_RES_17);
    mag.setResolution(MLX90393_Y, MLX90393_RES_17);
    mag.setResolution(MLX90393_Z, MLX90393_RES_17);
    mag.setOversampling(MLX90393_OSR_3);
    mag.setFilter(MLX90393_FILTER_5);
}

void Sensors::poll() {
    // IMU
    imu.getAGT();
    _vehicle->imu_ax = imu.accX();
    _vehicle->imu_ay = imu.accY();
    _vehicle->imu_az = imu.accZ();
    _vehicle->imu_gx = imu.gyrX();
    _vehicle->imu_gy = imu.gyrY();
    _vehicle->imu_gz = imu.gyrZ();
    _vehicle->imu_temp = imu.temp();

    // Compass
    mag.readData(&_vehicle->compass_mx, &_vehicle->compass_my, &_vehicle->compass_mz);

    // Barometer
    // if (baro.read()) {}
    baro.read();
    _vehicle->baro_alt = (pow(1013.25/baro.getPressure(), 1.0 / 5.257) - 1.0) * (baro.getTemperature() + 273.15) / 0.0065;

    // Power monitoring
    _vehicle->batt_voltage = analogRead(PC0) * (3.3 / 1023.0);
    _vehicle->batt_current = analogRead(PC2) * (3.3 / 1023.0);
    _vehicle->autopilot_voltage = ina219.read_voltage();
    _vehicle->autopilot_current = ina219.read_current();
}