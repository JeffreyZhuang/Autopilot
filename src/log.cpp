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
    if (file) {
        swo.print("Writing to datalog.txt...");
        file.println("testing 1, 2, 3.");
        file.flush();
        swo.println("done.");
    } else {
        swo.println("error opening test.txt");
    }

    // myFile.close();
    // myFile = SD.open("datalog.txt");
    // if (myFile) {
    //     swo.println("test.txt:");

    //     while (myFile.available()) {
    //         swo.write(myFile.read());
    //     }

    //     myFile.close();
    // } else {
    //     swo.println("error opening test.txt");
    // }
    // if (!SD.end()) {
    //     swo.println("Failed to properly end the SD.");
    // }
}

void DataLog::write() {
    file.println(String(millis()) + "," + String(_vehicle->baro_alt) + "," + String(_vehicle->imu_az));
    file.flush();
}