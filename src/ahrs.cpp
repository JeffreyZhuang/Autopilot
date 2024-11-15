#include <ahrs.h>

AHRS::AHRS(Vehicle * vehicle) {
    _vehicle = vehicle;
}

void AHRS::setup() {
    filter.begin(100);
}

void AHRS::update() {
    
}