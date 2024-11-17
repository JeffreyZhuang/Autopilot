#include <mlx90393.h>

MLX90393::MLX90393(SPIClass * spi_bus, uint8_t cs_pin, uint32_t spi_speed) {
    _spi_bus = spi_bus;
    _cs_pin = cs_pin;
    _spi_settings = SPISettings(spi_speed, MSBFIRST, SPI_MODE0);
}

void MLX90393::setup() {
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);

    exit_mode();
    reset();
    set_gain(MLX90393_GAIN_1X);
    set_res(MLX90393_X, MLX90393_RES_16);
    set_res(MLX90393_Y, MLX90393_RES_16);
    set_res(MLX90393_Z, MLX90393_RES_16);
    set_oversampling(MLX90393_OSR_3);
    set_filter(MLX90393_FILTER_7);
    start_burst_mode();
}

void MLX90393::reset() {
    command(0xF0);
}

void MLX90393::exit_mode() {
    command(0x80);
}

void MLX90393::start_burst_mode() {
    command(0x10 | 0x0E);
}

void MLX90393::set_gain(mlx90393_gain_t gain) {
    _gain = gain;

    uint16_t data;
    read_register(0x00, &data);

    data &= ~0x0070;
    data |= gain << 4;

    write_register(0x00, data);
}

void MLX90393::set_res(enum mlx90393_axis axis, enum mlx90393_resolution resolution) {
    uint16_t data;
    read_register(0x02, &data);

    switch (axis) {
    case MLX90393_X:
        _res_x = resolution;
        data &= ~0x0060;
        data |= resolution << 5;
        break;
    case MLX90393_Y:
        _res_y = resolution;
        data &= ~0x0180;
        data |= resolution << 7;
        break;
    case MLX90393_Z:
        _res_z = resolution;
        data &= ~0x0600;
        data |= resolution << 9;
        break;
    }

    write_register(0x02, data);
}

void MLX90393::set_oversampling(enum mlx90393_oversampling oversampling) {
    _osr = oversampling;

    uint16_t data;
    read_register(0x02, &data);

    data &= ~0x03;
    data |= oversampling;

    write_register(0x02, data);
}

void MLX90393::set_filter(enum mlx90393_filter filter) {
    _dig_filt = filter;

    uint16_t data;
    read_register(0x02, &data);

    data &= ~0x1C;
    data |= filter << 2;

    write_register(0x02, data);
}

void MLX90393::read_measurement(float *x, float *y, float *z) {
    uint8_t rx_len = 6;
    uint8_t rx_buf[rx_len];

    digitalWrite(_cs_pin, LOW);

    _spi_bus->beginTransaction(_spi_settings);
    _spi_bus->transfer(0x40 | 0x0E);
    _spi_bus->transfer(0x00);
    _spi_bus->transfer(0x00);
    _spi_bus->transfer(0x00);
    for (int i = 0; i < rx_len; i++) {
        rx_buf[i] = _spi_bus->transfer(0x00);
    }
    _spi_bus->endTransaction();

    digitalWrite(_cs_pin, HIGH);

    int16_t xi, yi, zi;

    xi = (rx_buf[0] << 8) | rx_buf[1];
    yi = (rx_buf[2] << 8) | rx_buf[3];
    zi = (rx_buf[4] << 8) | rx_buf[5];

    if (_res_x == MLX90393_RES_18)
        xi -= 0x8000;
    if (_res_x == MLX90393_RES_19)
        xi -= 0x4000;
    if (_res_y == MLX90393_RES_18)
        yi -= 0x8000;
    if (_res_y == MLX90393_RES_19)
        yi -= 0x4000;
    if (_res_z == MLX90393_RES_18)
        zi -= 0x8000;
    if (_res_z == MLX90393_RES_19)
        zi -= 0x4000;
    
    *x = (float)xi * mlx90393_lsb_lookup[0][_gain][_res_x][0];
    *y = (float)yi * mlx90393_lsb_lookup[0][_gain][_res_y][0];
    *z = (float)zi * mlx90393_lsb_lookup[0][_gain][_res_z][1];
}

void MLX90393::read_register(uint8_t reg, uint16_t *data) {
    uint8_t rx_buf[2];
    
    digitalWrite(_cs_pin, LOW);

    _spi_bus->beginTransaction(_spi_settings);
    _spi_bus->transfer(0x50);
    _spi_bus->transfer(reg << 2);
    rx_buf[0] = _spi_bus->transfer(0x00);
    rx_buf[1] = _spi_bus->transfer(0x00);
    _spi_bus->endTransaction();

    digitalWrite(_cs_pin, HIGH);

    *data = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
}

void MLX90393::write_register(uint8_t reg, uint16_t data) {
    digitalWrite(_cs_pin, LOW);

    _spi_bus->beginTransaction(_spi_settings);
    _spi_bus->transfer(0x60);
    _spi_bus->transfer(data >> 8);
    _spi_bus->transfer(data & 0xFF);
    _spi_bus->transfer(reg << 2);
    _spi_bus->endTransaction();

    digitalWrite(_cs_pin, HIGH);
}

void MLX90393::command(const uint8_t data) {
    digitalWrite(_cs_pin, LOW);

    _spi_bus->beginTransaction(_spi_settings);
    _spi_bus->transfer(data);
    _spi_bus->endTransaction();

    digitalWrite(_cs_pin, HIGH);
}