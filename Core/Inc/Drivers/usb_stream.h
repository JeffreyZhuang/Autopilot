#ifndef INC_DRIVERS_USB_STREAM_H_
#define INC_DRIVERS_USB_STREAM_H_

extern "C"
{
#include "ring_buffer.h"
#include "usbd_cdc_if.h"
}

class USB_stream
{
public:
	USB_stream();

	void transmit(uint8_t tx_buff[], int len);
	bool read_byte(uint8_t* byte);
	void rx_callback(uint8_t* Buf, uint32_t Len);

private:
	ring_buffer_t ring_buffer;
};

#endif /* INC_DRIVERS_USB_STREAM_H_ */
