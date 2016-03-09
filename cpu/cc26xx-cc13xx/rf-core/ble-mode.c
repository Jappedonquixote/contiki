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
#ifdef __GNUC__
#define CC_ALIGN_ATTR(n) __attribute__ ((aligned(n)))
#else
#define CC_ALIGN_ATTR(n)
#endif
/*---------------------------------------------------------------------------*/
#define DEBUG 0
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
/* The outgoing frame buffer */
#define TX_BUF_PAYLOAD_LEN  32
#define TX_BUF_HDR_LEN      24

static uint8_t tx_buf[TX_BUF_HDR_LEN + TX_BUF_PAYLOAD_LEN] CC_ALIGN_ATTR(4);
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
LPM_MODULE(cc26xx_rf_lpm_module, request, NULL, NULL, LPM_DOMAIN_NONE);
/*---------------------------------------------------------------------------*//*---------------------------------------------------------------------------*/
static void
soft_off(void)
{

	uint32_t cmd_status;
  	volatile rfc_radioOp_t *cmd = rf_core_get_last_radio_op();

  	PRINTF("ble-mode soft_off\n");

  	if(!rf_core_is_accessible()) {
  		return;
  	}

  	PRINTF("soft_off: Aborting 0x%04x, Status=0x%04x\n", cmd->commandNo,
         cmd->status);

  	/* Send a CMD_ABORT command to RF Core */
  	if(rf_core_send_cmd(CMDR_DIR_CMD(CMD_ABORT), &cmd_status) != RF_CORE_CMD_OK) {
  		PRINTF("soft_off: CMD_ABORT status=0x%08lx\n", cmd_status);
  		return;
  	}

  	while((cmd->status & RF_CORE_RADIO_OP_MASKED_STATUS) ==
        RF_CORE_RADIO_OP_MASKED_STATUS_RUNNING);
}
/*---------------------------------------------------------------------------*/
static uint8_t
soft_on(void)
{
	PRINTF("ble-mode soft_on\n");
	if(rf_radio_setup() != RF_CORE_CMD_OK) {
		PRINTF("on: radio_setup() failed\n");
		return RF_CORE_CMD_ERROR;
	}
	return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
static const rf_core_primary_mode_t mode_ble = {
  soft_off,
  soft_on,
};
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

	rf_core_primary_mode_register(&mode_ble);

	process_start(&rf_core_process, NULL);
	return 1;
}

/*---------------------------------------------------------------------------*/
static int
prepare(const void *payload, unsigned short payload_len)
{
	PRINTF("ble-mode prepare\n");
	int len = MIN(payload_len, TX_BUF_PAYLOAD_LEN);
	memcpy(tx_buf, payload, len);

	for(int i = 0; i < len; ++i) {
		PRINTF("%02X ", tx_buf[i]);
	}
	PRINTF("\n");

	return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
static int
transmit(unsigned short transmit_len)
{
	PRINTF("ble-mode transmit\n");
	return RADIO_TX_OK;
}
///*---------------------------------------------------------------------------*/
//static int
//transmit(unsigned short transmit_len)
//{
//  int ret;
//  uint8_t was_off = 0;
//  uint32_t cmd_status;
//  uint16_t stat;
//  uint8_t tx_active = 0;
//  rtimer_clock_t t0;
//  rfc_CMD_IEEE_TX_t cmd;
//
//  if(!rf_is_on()) {
//    was_off = 1;
//    if(on() != RF_CORE_CMD_OK) {
//      PRINTF("transmit: on() failed\n");
//      return RADIO_TX_ERR;
//    }
//  }
//
//  /*
//   * We are certainly not TXing a frame as a result of CMD_IEEE_TX, but we may
//   * be in the process of TXing an ACK. In that case, wait for the TX to finish
//   * or return after approx TX_WAIT_TIMEOUT
//   */
//  t0 = RTIMER_NOW();
//
//  do {
//    tx_active = transmitting();
//  } while(tx_active == 1 &&
//          (RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + TX_WAIT_TIMEOUT)));
//
//  if(tx_active) {
//    PRINTF("transmit: Already TXing and wait timed out\n");
//
//    if(was_off) {
//      off();
//    }
//
//    return RADIO_TX_COLLISION;
//  }
//
//  /* Send the CMD_IEEE_TX command */
//  rf_core_init_radio_op((rfc_radioOp_t *)&cmd, sizeof(cmd), CMD_IEEE_TX);
//
//  cmd.payloadLen = transmit_len;
//  cmd.pPayload = &tx_buf[TX_BUF_HDR_LEN];
//
//  /* Enable the LAST_FG_COMMAND_DONE interrupt, which will wake us up */
//  rf_core_cmd_done_en(true);
//
//  ret = rf_core_send_cmd((uint32_t)&cmd, &cmd_status);
//
//  if(ret) {
//    /* If we enter here, TX actually started */
//    ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
//    ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
//
//    /* Idle away while the command is running */
//    while((cmd.status & RF_CORE_RADIO_OP_MASKED_STATUS)
//          == RF_CORE_RADIO_OP_MASKED_STATUS_RUNNING) {
//      lpm_sleep();
//    }
//
//    stat = cmd.status;
//
//    if(stat == RF_CORE_RADIO_OP_STATUS_IEEE_DONE_OK) {
//      /* Sent OK */
//      RIMESTATS_ADD(lltx);
//      ret = RADIO_TX_OK;
//    } else {
//      /* Operation completed, but frame was not sent */
//      PRINTF("transmit: ret=%d, CMDSTA=0x%08lx, status=0x%04x\n", ret,
//             cmd_status, stat);
//      ret = RADIO_TX_ERR;
//    }
//  } else {
//    /* Failure sending the CMD_IEEE_TX command */
//    PRINTF("transmit: ret=%d, CMDSTA=0x%08lx, status=0x%04x\n",
//           ret, cmd_status, cmd.status);
//
//    ret = RADIO_TX_ERR;
//  }
//
//  /*
//   * Update ENERGEST state here, before a potential call to off(), which
//   * will correctly update it if required.
//   */
//  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
//  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
//
//  /*
//   * Disable LAST_FG_COMMAND_DONE interrupt. We don't really care about it
//   * except when we are transmitting
//   */
//  rf_core_cmd_done_dis();
//
//
//  if(was_off) {
//    off();
//  }
//
//  return ret;
//}
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
