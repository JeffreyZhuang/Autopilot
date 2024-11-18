#include <ICM42688.h>
#include <MS5611_SPI.h>
#include <Arduino.h>
#include <SWOStream.h>
#include <vehicle.h>
#include <ina219.h>
#include <Adafruit_MLX90393.h>

extern SWOStream swo;

// Navigation polls from sensors. Poll IMU data from AHRS.
// AP_NavEKF2
// Look at NavEKF2. DAL? Data abstraction layer? So, both AHRS and navigation read from sensor. Use DAL to handle FIFO buffer (I guess this is better because you can use one independent of the other)
// GCS: https://github.com/blauret/pyG5
class Sensors {
public:
    Sensors(Vehicle * vehicle);

    void setup();
    void poll();
private:
    Vehicle * _vehicle;

    // Peripherals
    SPIClass spi_bus;
    TwoWire i2c_bus;

    // Sensors
    ICM42688 imu;
    MS5611_SPI baro;
    Adafruit_MLX90393 mag;
    INA219 ina219;
};

// The class that actually does the polling is inherited. So it makes sure it has correct properties to fit with rest of program.