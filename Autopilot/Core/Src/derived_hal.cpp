#include "derived_hal.h"

Derived_hal::Derived_hal(Plane * plane) : HAL(plane),
										  imu(&hspi1, GPIOC, GPIO_PIN_15, SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_4),
										  ina219(&hi2c1, 0.01),
										  gnss(&huart3)
{
	_plane = plane;
}

void Derived_hal::setup()
{
	imu.begin();

	Barometer_init();
	Barometer_setOSR(OSR_4096);

	mag.begin_SPI(GPIOC, GPIO_PIN_13, &hspi1, SPI_BAUDRATEPRESCALER_8);

	sd.initialize();

	gnss.setup();

	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
	{
		Error_Handler();
	}
}

void Derived_hal::poll()
{
	uint32_t time = HAL_GetTick();

	float alt = Barometer_getAltitude(true);
	_plane->baro_alt = alt;
	_plane->baro_timestamp = get_time_us();

	float voltage = ina219.read_voltage();
	float current = ina219.read_current();
	_plane->autopilot_voltage = voltage;
	_plane->autopilot_current = current;

	imu.getAGT();
	_plane->imu_ax = -imu.accX();
	_plane->imu_ay = -imu.accY();
	_plane->imu_az = imu.accZ();
	_plane->imu_gx = -imu.gyrX();
	_plane->imu_gy = -imu.gyrY();
	_plane->imu_gz = imu.gyrZ();
	_plane->imu_temp = imu.temp();
	_plane->imu_timestamp = get_time_us();

	mag.readDataNonBlocking();

	Sd_packet p;
	p.time = time;
	p.acc_z = imu.accZ();
	p.alt = alt;
	sd.append_buffer(p);

	uint8_t sentence[100];
	if (gnss.parse(sentence))
	{
		_plane->lat = gnss.lat;
		_plane->lon = gnss.lon;
	}
}

void Derived_hal::write_sd()
{
	sd.write();
}

void Derived_hal::read_sd()
{
	sd.read();
}

void Derived_hal::swo_print(char * str)
{

}

void Derived_hal::usb_print(char * str)
{

}

void Derived_hal::i2c_scan()
{

}

// Counter increments every 10 us
// Resolution is 10 us
// Maximum allowed time is 42949672950 us or 12 hours
// SysTick has maximum time of 4.7 hours
uint64_t Derived_hal::get_time_us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim5) * 10;
}

void Derived_hal::delay_us(uint64_t us)
{
	uint64_t start = get_time_us();
	while (get_time_us() - start < us);
}

void Derived_hal::toggle_led()
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5); // LED
}

void Derived_hal::gnss_callback()
{
	gnss.callback();
}
