// https://github.com/RobTillaart/MS5611_SPI

#include "MS5611_SPI.h"

#define MS5611_CMD_READ_ADC       0x00
#define MS5611_CMD_READ_PROM      0xA0
#define MS5611_CMD_RESET          0x1E
#define MS5611_CMD_CONVERT_D1     0x40
#define MS5611_CMD_CONVERT_D2     0x50

MS5611_SPI::MS5611_SPI(uint8_t select, SPIClass * mySPI)
{
  _samplingRate      = OSR_ULTRA_LOW;
  _temperature       = MS5611_NOT_READ;
  _pressure          = MS5611_NOT_READ;
  _result            = MS5611_NOT_READ;
  _select   = select;
  _mySPI    = mySPI;
}

bool MS5611_SPI::begin()
{
  pinMode(_select, OUTPUT);
  digitalWrite(_select, HIGH);

  setSPIspeed(_SPIspeed);

  return reset();
}

bool MS5611_SPI::reset(uint8_t mathMode)
{
  command(MS5611_CMD_RESET);
  delayMicroseconds(3000);

  initConstants(mathMode);

  bool ROM_OK = true;
  for (uint8_t reg = 0; reg < 7; reg++)
  {
    uint16_t tmp = readProm(reg);
    C[reg] *= tmp;

    if (reg > 0)
    {
      ROM_OK = ROM_OK && (tmp != 0);
    }
  }
  
  return ROM_OK;
}

int MS5611_SPI::read(uint8_t bits)
{
  convert(MS5611_CMD_CONVERT_D1, bits);
  uint32_t _D1 = readADC();

  convert(MS5611_CMD_CONVERT_D2, bits);
  uint32_t _D2 = readADC();

  float dT = _D2 - C[5];
  _temperature = 2000 + dT * C[6];

  float offset =  C[2] + dT * C[4];
  float sens = C[1] + dT * C[3];

  if (_temperature < 2000)
  {
    float T2 = dT * dT * 4.6566128731E-10;
    float t = (_temperature - 2000) * (_temperature - 2000);
    float offset2 = 2.5 * t;
    float sens2 = 1.25 * t;

    if (_temperature < -1500)
    {
      t = (_temperature + 1500) * (_temperature + 1500);
      offset2 += 7 * t;
      sens2 += 5.5 * t;
    }
    _temperature -= T2;
    offset -= offset2;
    sens -= sens2;
  }

  _pressure = (_D1 * sens * 4.76837158205E-7 - offset) * 3.051757813E-5;

  return MS5611_READ_OK;
}

void MS5611_SPI::setOversampling(osr_t samplingRate)
{
  _samplingRate = (uint8_t) samplingRate;
}

float MS5611_SPI::getTemperature() const
{
  return _temperature * 0.01;
}

float MS5611_SPI::getPressure() const
{
  return _pressure * 0.01;
}

void MS5611_SPI::setSPIspeed(uint32_t speed)
{
  _SPIspeed = speed;
  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);
}

void MS5611_SPI::convert(const uint8_t addr, uint8_t bits)
{
  uint16_t del[5] = {600, 1200, 2300, 4600, 9100};

  uint8_t index = bits;
  if (index < 8) index = 8;
  else if (index > 12) index = 12;
  index -= 8;
  uint8_t offset = index * 2;
  command(addr + offset);

  uint16_t waitTime = del[index];
  delayMicroseconds(waitTime);
}

uint16_t MS5611_SPI::readProm(uint8_t reg)
{
  uint8_t promCRCRegister = 7;
  if (reg > promCRCRegister) return 0;

  uint16_t value = 0;
  digitalWrite(_select, LOW);

  _mySPI->beginTransaction(_spi_settings);
  _mySPI->transfer(MS5611_CMD_READ_PROM + reg * 2);
  value += _mySPI->transfer(0x00);
  value <<= 8;
  value += _mySPI->transfer(0x00);
  _mySPI->endTransaction();

  digitalWrite(_select, HIGH);
  return value;
}

uint32_t MS5611_SPI::readADC()
{
  uint32_t value = 0;

  digitalWrite(_select, LOW);

  _mySPI->beginTransaction(_spi_settings);
  _mySPI->transfer(0x00);
  value += _mySPI->transfer(0x00);
  value <<= 8;
  value += _mySPI->transfer(0x00);
  value <<= 8;
  value += _mySPI->transfer(0x00);
  _mySPI->endTransaction();

  digitalWrite(_select, HIGH);

  return value;
}

void MS5611_SPI::command(const uint8_t command)
{
  digitalWrite(_select, LOW);

  _mySPI->beginTransaction(_spi_settings);
  _mySPI->transfer(command);
  _mySPI->endTransaction();

  digitalWrite(_select, HIGH);
}

void MS5611_SPI::initConstants(uint8_t mathMode)
{
  C[0] = 1;
  C[1] = 32768L;
  C[2] = 65536L;
  C[3] = 3.90625E-3; 
  C[4] = 7.8125E-3;   
  C[5] = 256;         
  C[6] = 1.1920928955E-7; 

  if (mathMode == 1)     
  {
    C[1] = 65536L;        
    C[2] = 131072L;      
    C[3] = 7.8125E-3;  
    C[4] = 1.5625e-2;  
  }
}