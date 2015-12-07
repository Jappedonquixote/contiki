#include "dev/radio.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
static int
init(void)
{
	PRINTF("ble-mode init\n");
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
prepare(const void *payload, unsigned short payload_len)
{
	PRINTF("ble-mode prepare\n");
	return 1;
}
/*---------------------------------------------------------------------------*/
static int
transmit(unsigned short transmit_len)
{
	PRINTF("ble-mode transmit\n");
	return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
send(const void *payload, unsigned short payload_len)
{
	PRINTF("ble-mode send called\n");
	prepare(payload, payload_len);
	return transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
static int
radio_read(void *buf, unsigned short buf_len)
{
	PRINTF("ble-mode radio_read\n");
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
channel_clear(void)
{
	PRINTF("ble-mode channel_clear\n");
	return 1;
}
/*---------------------------------------------------------------------------*/
static int
receiving_packet(void)
{
	PRINTF("ble-mode receiving_packet\n");
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
	PRINTF("ble-mode pending_packet\n");
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
	PRINTF("ble-mode on\n");
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
	PRINTF("ble-mode off\n");
	return 0;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
get_value(radio_param_t param, radio_value_t *value)
{
	PRINTF("ble-mode get_value\n");
	return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
set_value(radio_param_t param, radio_value_t value)
{
	PRINTF("ble-mode set_value\n");
	return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
get_object(radio_param_t param, void *dest, size_t size)
{
	PRINTF("ble-mode get_object\n");
	return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
set_object(radio_param_t param, const void *src, size_t size)
{
	PRINTF("ble-mode set_object\n");
	return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
const struct radio_driver ble_mode_driver =
  {
    init,
    prepare,
    transmit,
    send,
    radio_read,
    channel_clear,
    receiving_packet,
    pending_packet,
    on,
    off,
    get_value,
    set_value,
    get_object,
    set_object
  };
/*---------------------------------------------------------------------------*/
