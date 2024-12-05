#include <hal_arduino.h>

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

    file = SD.open("datalog.txt", FA_WRITE | FA_OPEN_ALWAYS);
    if (!file) {
        swo.print("error opening test.txt\n");
    }
}

/**
 * @brief Write to micro SD card
 * 
 */
void HAL_Arduino::write_sd() {
    file.println(String(millis()) + "," + String(_plane->baro_alt, 2) + "," + String(_plane->imu_az, 1) + "," + String(_plane->imu_gz, 1));
    num_writes++;

    // if (num_writes > 10) {
        file.flush();
        num_writes = 0;
    // }   
}