#include <main.h>

Vehicle vehicle;
AHRS ahrs(&vehicle);
Sensors sensors(&vehicle);
Navigation navigation(&vehicle);
DataLog data_log(&vehicle);
SWOStream swo(2000000);
uint32_t prev_loop_time;
uint32_t prev_print_time;

void setup_leds() {
  pinMode(PC1, OUTPUT);
}

void setup() {
  data_log.setup(); // BUG: Does not work when moved under i2c_begin. I haven't defined the SDIO pins, maybe pins override i2c?

  setup_leds();

  ahrs.setup();
  sensors.setup();

  digitalWrite(PC1, HIGH);
  delay(500);
  digitalWrite(PC1, LOW);
  delay(500);
}

void loop() {
  sensors.poll();
  ahrs.update();
  navigation.update();
  data_log.write();

  if (micros() - prev_print_time > 50000) {
    // Replace with USB instead of SWO
    // swo.println("Time: " + String(millis()));
    swo.println(String(vehicle.baro_alt) + "\t" + String(vehicle.imu_ax) + "\t" + String(vehicle.imu_ay) + 
                "\t" + String(vehicle.imu_gz) + "\t" + String(vehicle.compass_mx) + "\t" + 
                String(vehicle.compass_my) + "\t" + String(vehicle.compass_mz));
    // swo.println(vehicle.autopilot_voltage, 6);
    // swo.println(vehicle.autopilot_current, 6);
    // swo.println(vehicle.batt_voltage, 6);
    // swo.println(vehicle.batt_current, 6);
    swo.println("dt: " + String((micros() - prev_loop_time) / 1000));
    prev_print_time = micros();
  }
  
  prev_loop_time = micros();
}

// #include <Adafruit_MLX90393.h>
// #include <SWOStream.h>
// #include <Arduino.h>

// Adafruit_MLX90393 sensor = Adafruit_MLX90393();
// SPIClass spi(PB5, PB4, PA5);

// SWOStream swo(2000000);

// void setup() {
//   if (!sensor.begin_SPI(PC13, &spi)) {
//     swo.println("No sensor found");
//   }

//   sensor.setGain(MLX90393_GAIN_1X);
//   sensor.setResolution(MLX90393_X, MLX90393_RES_16);
//   sensor.setResolution(MLX90393_Y, MLX90393_RES_16);
//   sensor.setResolution(MLX90393_Z, MLX90393_RES_16);
//   sensor.setOversampling(MLX90393_OSR_3);
//   sensor.setFilter(MLX90393_FILTER_7);
// }

// void loop() {
//   float x, y, z;
//   if (sensor.readData(&x, &y, &z)) {
//       swo.print(x);
//       swo.print("\t");
//       swo.print(y);
//       swo.print("\t");
//       swo.println(z);
//   } else {
//       swo.println("Unable to read");
//   }
// }