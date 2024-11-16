//
//    FILE: MS5611_SPI.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.3.1
// PURPOSE: MS5611 (SPI) Temperature & Pressure library for Arduino
//     URL: https://github.com/RobTillaart/MS5611_SPI
//
//  HISTORY: see changelog.md

#include "MS5611_SPI.h"

// datasheet page 10
#define MS5611_CMD_READ_ADC       0x00
#define MS5611_CMD_READ_PROM      0xA0
#define MS5611_CMD_RESET          0x1E
#define MS5611_CMD_CONVERT_D1     0x40
#define MS5611_CMD_CONVERT_D2     0x50

/////////////////////////////////////////////////////
//
//  PUBLIC
//
MS5611_SPI::MS5611_SPI(uint8_t select, __SPI_CLASS__ * mySPI)
{
  _samplingRate      = OSR_ULTRA_LOW;
  _temperature       = MS5611_NOT_READ;
  _pressure          = MS5611_NOT_READ;
  _result            = MS5611_NOT_READ;
  _lastRead          = 0;
  _pressureOffset    = 0;
  _temperatureOffset = 0;
  _compensation      = true;

  //  SPI
  _select   = select;
  _dataIn   = 255;
  _dataOut  = 255;
  _clock    = 255;
  _mySPI    = mySPI;
}

MS5611_SPI::MS5611_SPI(uint8_t select, uint8_t dataOut, uint8_t dataIn, uint8_t clock)
{
  _samplingRate      = OSR_ULTRA_LOW;
  _temperature       = MS5611_NOT_READ;
  _pressure          = MS5611_NOT_READ;
  _result            = MS5611_NOT_READ;
  _lastRead          = 0;
  _pressureOffset    = 0;
  _temperatureOffset = 0;
  _compensation      = false;

  //  SPI
  _select   = select;
  _dataIn   = dataIn;
  _dataOut  = dataOut;
  _clock    = clock;
  _mySPI    = NULL;
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
  uint32_t start = micros();
  //  while loop prevents blocking RTOS
  while (micros() - start < 3000)     //  increased as first ROM values were missed.
  {
    yield();
    delayMicroseconds(10);
  }

  //  initialize the C[] array
  initConstants(mathMode);

  //  read factory calibrations from EEPROM.
  bool ROM_OK = true;
  for (uint8_t reg = 0; reg < 7; reg++)
  {
    //  used indices match datasheet.
    //  C[0] == manufacturer - read but not used;
    //  C[7] == CRC - skipped.
    uint16_t tmp = readProm(reg);
    C[reg] *= tmp;
    //  Serial.println(readProm(reg));
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

  if (_compensation)
  {
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
  }

  _pressure = (_D1 * sens * 4.76837158205E-7 - offset) * 3.051757813E-5;

  _lastRead = millis();
  return MS5611_READ_OK;
}

void MS5611_SPI::setOversampling(osr_t samplingRate)
{
  _samplingRate = (uint8_t) samplingRate;
}

float MS5611_SPI::getTemperature() const
{
  if (_temperatureOffset == 0) return _temperature * 0.01;
  return _temperature * 0.01 + _temperatureOffset;
}

float MS5611_SPI::getPressure() const
{
  if (_pressureOffset == 0) return _pressure * 0.01;
  return _pressure * 0.01 + _pressureOffset;
}

void MS5611_SPI::setSPIspeed(uint32_t speed)
{
  _SPIspeed = speed;
  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);
}

/////////////////////////////////////////////////////
//
//  PRIVATE
//
void MS5611_SPI::convert(const uint8_t addr, uint8_t bits)
{
  //  values from page 3 datasheet - MAX column (rounded up)
  uint16_t del[5] = {600, 1200, 2300, 4600, 9100};

  uint8_t index = bits;
  if (index < 8) index = 8;
  else if (index > 12) index = 12;
  index -= 8;
  uint8_t offset = index * 2;
  command(addr + offset);

  uint16_t waitTime = del[index];
  uint32_t start = micros();
  //  while loop prevents blocking RTOS
  while (micros() - start < waitTime)
  {
    yield();
    delayMicroseconds(10);
  }
}

uint16_t MS5611_SPI::readProm(uint8_t reg)
{
  //  last EEPROM register is CRC - Page 13 datasheet.
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
  //  command(MS5611_CMD_READ_ADC);

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

int MS5611_SPI::command(const uint8_t command)
{
  yield();
  digitalWrite(_select, LOW);

  _mySPI->beginTransaction(_spi_settings);
  _mySPI->transfer(command);
  _mySPI->endTransaction();

  digitalWrite(_select, HIGH);
  return 0;
}

void MS5611_SPI::initConstants(uint8_t mathMode)
{
  //  constants that were multiplied in read() - datasheet page 8
  //  do this once and you save CPU cycles
  //
  //                               datasheet ms5611     |    appNote
  //                                mode = 0;           |    mode = 1
  C[0] = 1;
  C[1] = 32768L;          //  SENSt1   = C[1] * 2^15    |    * 2^16
  C[2] = 65536L;          //  OFFt1    = C[2] * 2^16    |    * 2^17
  C[3] = 3.90625E-3;      //  TCS      = C[3] / 2^8     |    / 2^7
  C[4] = 7.8125E-3;       //  TCO      = C[4] / 2^7     |    / 2^6
  C[5] = 256;             //  Tref     = C[5] * 2^8     |    * 2^8
  C[6] = 1.1920928955E-7; //  TEMPSENS = C[6] / 2^23    |    / 2^23

  if (mathMode == 1)      //  Appnote version for pressure.
  {
    C[1] = 65536L;        //  SENSt1
    C[2] = 131072L;       //  OFFt1
    C[3] = 7.8125E-3;     //  TCS
    C[4] = 1.5625e-2;     //  TCO
  }
}