/**
 * \file
 * 		   Bluetooth LE radio driver
 * \author
 *         Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "contiki.h"
#include "dev/radio.h"
#include "lpm.h"
#include "rf-core/ble-stack/ble-controller.h"
/*---------------------------------------------------------------------------*/
#ifdef __GNUC__
#define CC_ALIGN_ATTR(n) __attribute__ ((aligned(n)))
#else
#define CC_ALIGN_ATTR(n)
#endif
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
static int on(void);
static int off(void);

/*---------------------------------------------------------------------------*/
static uint8_t
request(void)
{
//    PRINTF("[ ble-phy ] request()\n");
	if(ble_controller_is_enabled()) {
		return LPM_MODE_SLEEP;
	}

	return LPM_MODE_MAX_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
LPM_MODULE(cc26xx_rf_lpm_module, request, NULL, NULL, LPM_DOMAIN_NONE);
/*---------------------------------------------------------------------------*/
static int
init(void)
{
    int result;
	PRINTF("[ ble-phy ] init()\n");

	lpm_register_module(&cc26xx_rf_lpm_module);

	result = ble_controller_reset();
	if(result != BLE_COMMAND_SUCCESS)
	{
	    PRINTF("[ ble-phy ] init(): could not reset ble controller\n");
	}

	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
	return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
static int
prepare(const void *payload, unsigned short payload_len)
{
	PRINTF("ble-mode prepare\n");
	return BLE_COMMAND_SUCCESS;
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
	PRINTF("[ ble-phy ] on()\n");
	if(ble_controller_enable() != BLE_COMMAND_SUCCESS)
	{
	    PRINTF("[ ble-phy ] on(): could not enable ble controller\n");
	    return BLE_COMMAND_ERROR;
	}
	return BLE_COMMAND_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
	PRINTF("[ ble-phy ] off()\n");
	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
	if(ble_controller_disable() != BLE_COMMAND_SUCCESS)
	{
	    PRINTF("[ ble-phy ] off(): could not disable ble controller\n");
	    return BLE_COMMAND_ERROR;
	}
	return BLE_COMMAND_SUCCESS;
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
