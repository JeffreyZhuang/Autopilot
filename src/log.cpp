#include <log.h>

DataLog::DataLog(Vehicle * vehicle) {
    _vehicle = vehicle;
}

void DataLog::setup() {
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

void DataLog::write() {
    file.println(String(micros()) + "," + String(_vehicle->baro_alt) + "," + String(_vehicle->imu_az));
    file.flush();
}