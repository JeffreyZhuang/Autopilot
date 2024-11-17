#include <Adafruit_MLX90393.h>
#include <SWOStream.h>
#include <Arduino.h>

Adafruit_MLX90393 sensor = Adafruit_MLX90393();

SWOStream swo(2000000);

void setup() {
    if (!sensor.begin_SPI(PC13)) {
        swo.println("No sensor found");
    }

    sensor.setGain(MLX90393_GAIN_1X);
    sensor.setResolution(MLX90393_X, MLX90393_RES_16);
    sensor.setResolution(MLX90393_Y, MLX90393_RES_16);
    sensor.setResolution(MLX90393_Z, MLX90393_RES_16);

    sensor.setOversampling(MLX90393_OSR_3);

    sensor.setFilter(MLX90393_FILTER_7);
}

void loop() {
    float x, y, z;
    if (sensor.readData(&x, &y, &z)) {
        swo.print(x);
        swo.print("\t");
        swo.print(y);
        swo.print("\t");
        swo.println(z);
    } else {
        swo.println("Unable to read");
    }
}