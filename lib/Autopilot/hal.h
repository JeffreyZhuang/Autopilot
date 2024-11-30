#include <ICM42688.h>
#include <MS5611_SPI.h>
#include <Arduino.h>
#include <SWOStream.h>
#include <plane.h>
#include <ina219.h>
#include <Adafruit_MLX90393.h>
#include <STM32SD.h>

// Hardware abstraction layer
// All hardware dependent code goes here

extern SWOStream swo;

// Navigation polls from sensors. Poll IMU data from AHRS.
// AP_NavEKF2
// Look at NavEKF2. DAL? Data abstraction layer? So, both AHRS and navigation read from sensor. Use DAL to handle FIFO buffer (I guess this is better because you can use one independent of the other)
// GCS: https://github.com/blauret/pyG5
class HAL {
public:
    HAL(Plane * plane);

    void setup();
    void poll();
    void blink_led();
    void delay_us(uint32_t us);
    void swo_print(char * str);
    void usb_print(char * str);
    void write_sd();
    uint32_t get_time_us();
private:
    void setup_peripherals();
    void setup_sensors();
    void setup_sd();
    void poll_imu();
    void poll_compass();
    void poll_barometer();
    void poll_power_monitor();

    Plane * _plane;

    // Peripherals
    SPIClass spi_bus;
    TwoWire i2c_bus;
    SWOStream swo;

    // Sensors
    ICM42688 imu;
    MS5611_SPI baro;
    Adafruit_MLX90393 mag;
    INA219 ina219;

    // Micro SD
    Sd2Card card;
    SdFatFs fatFs;
    File file;
};

// The class that actually does the polling is inherited. So it makes sure it has correct properties to fit with rest of program.