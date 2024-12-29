#include "derived_hal.h"

void Derived_hal::init_gnss()
{
	gnss.setup();
}

void Derived_hal::read_gnss()
{
	uint8_t sentence[100];
	if (gnss.parse(sentence))
	{
		_plane->lat = gnss.lat;
		_plane->lon = gnss.lon;
	}
}

void Derived_hal::gnss_dma_complete()
{
	gnss.dma_complete();
}
