#include <ina219.h>

INA219::INA219(TwoWire * i2c_bus, uint8_t i2c_address, float shunt_resistance) {
    _i2c_bus = i2c_bus;
    _i2c_address = i2c_address;
    _shunt_resistance = shunt_resistance;
}

uint16_t INA219::read_register(uint8_t reg) {
    _i2c_bus->beginTransmission(_i2c_address);
    _i2c_bus->write(reg);
    _i2c_bus->endTransmission(false);
    _i2c_bus->requestFrom(_i2c_address, static_cast<uint8_t>(2));
    uint16_t value = _i2c_bus->read() & 0xFF;
    value <<= 8;  // Move to upper byte
    value |= _i2c_bus->read() & 0xFF;
    return value;
}

float INA219::read_voltage() {
    return (float)(read_register(0x02) >> 3) * 0.004;
}

float INA219::read_current() {
    return (float)(read_register(0x01)) * 0.00001 / _shunt_resistance;
}