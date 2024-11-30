#include <ICM42688.h>
#include <MS5611_SPI.h>
#include <Arduino.h>
#include <SWOStream.h>
#include <plane.h>
#include <ina219.h>
#include <Adafruit_MLX90393.h>
#include <STM32SD.h>
#include <hal.h>

class HAL_Arduino : public HAL {
public:
    HAL_Arduino(Plane * plane);

    void setup();
    void poll();
    void toggle_led();
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