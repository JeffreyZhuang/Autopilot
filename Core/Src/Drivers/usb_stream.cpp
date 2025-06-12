#include "Drivers/usb_stream.h"

#define RING_BUFFER_SIZE (1024) // Must be a power of 2

static uint8_t data_buffer[RING_BUFFER_SIZE] = {0U};

USB_stream::USB_stream()
{
	ring_buffer_setup(&ring_buffer, data_buffer, RING_BUFFER_SIZE);
}

void USB_stream::transmit(uint8_t tx_buff[], int len)
{
	// NOTE: You must buffer and send everything in one transmit
	// If you call transmit right after another transmit,
	// the USB will be busy waiting for the first transmit
	// to complete and the second transmit will not be executed

	// Better way is to add to ring buffer
	// On transmit completion interrupt, empty the
	// rest of the buffer if its not empty yet

	// A while loop that blocks until USB is ready doesn't work
	// because if USB is not connected it will block forever
	CDC_Transmit_FS(tx_buff, len);
}

bool USB_stream::read_byte(uint8_t* byte)
{
	return ring_buffer_read(&ring_buffer, byte);
}

void USB_stream::rx_callback(uint8_t* Buf, uint32_t Len)
{
	for (uint32_t i = 0; i < Len; i++)
	{
		ring_buffer_write(&ring_buffer, Buf[i]);
	}
}

bool USB_stream::buffer_empty()
{
	return ring_buffer_empty(&ring_buffer);
}
