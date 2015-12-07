/**
 * \file
 * 		   Bluetooth LE radio driver
 * \author
 *         Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "contiki.h"
#include "dev/radio.h"
#include "dev/cc26xx-uart.h"
#include "dev/oscillators.h"

#include "lpm.h"

#include "ti-lib.h"
#include "rf-core/rf-core.h"
#include "rf-core/rf-ble.h"
/*---------------------------------------------------------------------------*/
/* RF core and RF HAL API */
#include "hw_rfc_dbell.h"
#include "hw_rfc_pwr.h"
/*---------------------------------------------------------------------------*/
/* RF Core Mailbox API */
#include "rf-core/api/mailbox.h"
#include "rf-core/api/common_cmd.h"
#include "rf-core/api/ieee_cmd.h"
#include "rf-core/api/data_entry.h"
#include "rf-core/api/ieee_mailbox.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
typedef struct default_ble_tx_power_s {
   uint16_t ib:6;
   uint16_t gc:2;
   uint16_t boost:1;
   uint16_t temp_coeff:7;
} default_ble_tx_power_t;

static default_ble_tx_power_t tx_power = { 0x29, 0x00, 0x00, 0x00 };
/*---------------------------------------------------------------------------*/
/* BLE overrides */
static uint32_t ble_overrides[] = {
  0x00364038, /* Synth: Set RTRIM (POTAILRESTRIM) to 6 */
  0x000784A3, /* Synth: Set FREF = 3.43 MHz (24 MHz / 7) */
  0xA47E0583, /* Synth: Set loop bandwidth after lock to 80 kHz (K2) */
  0xEAE00603, /* Synth: Set loop bandwidth after lock to 80 kHz (K3, LSB) */
  0x00010623, /* Synth: Set loop bandwidth after lock to 80 kHz (K3, MSB) */
  0x00456088, /* Adjust AGC reference level */
  0xFFFFFFFF, /* End of override list */
};
/*---------------------------------------------------------------------------*/
static int on(void);
static int off(void);

/*---------------------------------------------------------------------------*/
/**
 * \brief Checks whether the RFC domain is accessible and the RFC is in IEEE RX
 * \return 1: RFC in RX mode (and therefore accessible too). 0 otherwise
 */
static uint8_t
rf_is_on(void)
{
	if(!rf_core_is_accessible()) {
		return 0;
	}
	else {
		return 1;
	}
//  return RF_RADIO_OP_GET_STATUS(cmd_ieee_rx_buf) == RF_CORE_RADIO_OP_STATUS_ACTIVE;
}
/*---------------------------------------------------------------------------*/
static uint8_t
request(void)
{
	/*
	 * We rely on the RDC layer to turn us on and off. Thus, if we are on we
	 * will only allow sleep, standby otherwise
	 */
	if(rf_is_on()) {
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
	PRINTF("ble-mode init\n");
	lpm_register_module(&cc26xx_rf_lpm_module);

	rf_core_set_modesel();

	/* Populate the RF parameters data structure with default values */
//	init_rf_params();

	if(on() != RF_CORE_CMD_OK) {
		PRINTF("init: on() failed\n");
		return RF_CORE_CMD_ERROR;
	}

	ENERGEST_ON(ENERGEST_TYPE_LISTEN);
//
//	rf_core_primary_mode_register(&mode_ble);

	process_start(&rf_core_process, NULL);
	return 1;
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
static uint8_t
rf_radio_setup()
{
	uint32_t cmd_status;
	rfc_CMD_RADIO_SETUP_t cmd;

	/* Create radio setup command */
	rf_core_init_radio_op((rfc_radioOp_t *)&cmd, sizeof(cmd), CMD_RADIO_SETUP);

	cmd.txPower.IB = tx_power.ib;
	cmd.txPower.GC = tx_power.gc;
	cmd.txPower.tempCoeff = tx_power.temp_coeff;
	cmd.txPower.boost = tx_power.boost;
	cmd.pRegOverride = ble_overrides;
	cmd.mode = 0;

	/* Send Radio setup to RF Core */
	if(rf_core_send_cmd((uint32_t)&cmd, &cmd_status) != RF_CORE_CMD_OK) {
		PRINTF("rf_radio_setup: CMDSTA=0x%08lx, status=0x%04x\n",
				cmd_status, cmd.status);
		return RF_CORE_CMD_ERROR;
	}

	/* Wait until radio setup is done */
	if(rf_core_wait_cmd_done(&cmd) != RF_CORE_CMD_OK) {
		PRINTF("rf_radio_setup: wait, CMDSTA=0x%08lx, status=0x%04x\n",
           cmd_status, cmd.status);
		return RF_CORE_CMD_ERROR;
	}

	return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
	PRINTF("ble-mode on\n");
	/*
	 * Request the HF XOSC as the source for the HF clock. Needed before we can
	 * use the FS. This will only request, it will _not_ perform the switch.
	 */
	oscillators_request_hf_xosc();

	if(rf_is_on()) {
		return RF_CORE_CMD_OK;
	}

	if(rf_core_boot() != RF_CORE_CMD_OK) {
		PRINTF("on: rf_core_boot() failed\n");
		return RF_CORE_CMD_ERROR;
	}

	rf_core_setup_interrupts();

	/*
	 * Trigger a switch to the XOSC, so that we can subsequently use the RF FS
	 * This will block until the XOSC is actually ready, but give how we
	 * requested it early on, this won't be too long a wait/
	 */
	oscillators_switch_to_hf_xosc();

	if(rf_radio_setup() != RF_CORE_CMD_OK) {
		PRINTF("on: radio_setup() failed\n");
		return RF_CORE_CMD_ERROR;
	}

	return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
	PRINTF("ble-mode off\n");

//	while (transmitting());
	rf_core_power_down();

	ENERGEST_OFF(ENERGEST_TYPE_LISTEN);

	/* Switch HF clock source to the RCOSC to preserve power */
	oscillators_switch_to_hf_rc();

	return RF_CORE_CMD_OK;
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
