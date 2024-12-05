#include <hal_arduino.h>

/**
 * @brief Setup compass
 * 
 */
void HAL_Arduino::setup_compass() {
    mag.begin_SPI(PC13, &spi_bus);
    mag.setGain(MLX90393_GAIN_1X);
    mag.setResolution(MLX90393_X, MLX90393_RES_16);
    mag.setResolution(MLX90393_Y, MLX90393_RES_16);
    mag.setResolution(MLX90393_Z, MLX90393_RES_16);
    mag.setOversampling(MLX90393_OSR_3);
    mag.setFilter(MLX90393_FILTER_7);
}

/**
 * @brief Send magnetometer data through USB CDC for MotionCal
 * 
 */
void HAL_Arduino::calibrate_compass() {
    for (;;) {
        static float x, y, z;

        // Sample data from magnetometer
        if (mag.readData(&x, &y, &z)) {

            // Print raw data out for MotionCal (no accel or gyro data)
            Serial.print("Raw:");
            Serial.print(0); Serial.print(",");
            Serial.print(0); Serial.print(",");
            Serial.print(0); Serial.print(",");
            Serial.print(0); Serial.print(",");
            Serial.print(0); Serial.print(",");
            Serial.print(0); Serial.print(",");
            Serial.print(int(x*10)); Serial.print(",");
            Serial.print(int(y*10)); Serial.print(",");
            Serial.print(int(z*10)); Serial.println("");

            // Unified data (no accel or gyro data)
            Serial.print("Uni:");
            Serial.print(0); Serial.print(",");
            Serial.print(0); Serial.print(",");
            Serial.print(0); Serial.print(",");
            Serial.print(0); Serial.print(",");
            Serial.print(0); Serial.print(",");
            Serial.print(0); Serial.print(",");
            Serial.print(int(x)); Serial.print(",");
            Serial.print(int(y)); Serial.print(",");
            Serial.print(int(z)); Serial.println("");

        } else {
            Serial.println("Unable to read XYZ data from the sensor.");
        }

        // Wait some
        delay(10);
    }    
}

/**
 * @brief Poll data from compass and update plane struct
 * 
 */
void HAL_Arduino::poll_compass() {
    float hi_cal[3];
    float hard_iron[3] = {52.67, -5.27, 81.54};
    float soft_iron[3][3] = {{1.031, 0.015, -0.0032},
                                   {0.015, 0.967, -0.025},
                                   {-0.032, -0.025, 1.005}};
    
    if (mag.readDataNonBlocking()) { // The issue with the previous nonblocking read data function is because if there is no data, value of x gets set to 0 when you pass pointer since nothing gets set
        _plane->compass_timestamp = get_time_us();

        float mag_data[3] = {mag.x, mag.y, mag.z};
        
        // Apply hard-iron offsets
        for (uint8_t i = 0; i < 3; i++) {
            hi_cal[i] = mag_data[i] - hard_iron[i];
        }

        // Apply soft-iron scaling
        for (uint8_t i = 0; i < 3; i++) {
            mag_data[i] = (soft_iron[i][0] * hi_cal[0]) + 
                          (soft_iron[i][1] * hi_cal[1]) +
                          (soft_iron[i][2] * hi_cal[2]);
        }

        _plane->compass_mx = -mag_data[0];
        _plane->compass_my = mag_data[1];
        _plane->compass_mz = -mag_data[2];
    }
}