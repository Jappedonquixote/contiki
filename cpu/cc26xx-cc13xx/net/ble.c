/*
 * ble.c
 *
 *  Created on: 07. Dez. 2015
 *      Author: Michi
 */

#include "ble.h"
#include "net/netstack.h"
#include "net/linkaddr.h"
#include "rf-core/rf-core.h"
#include "rf-core/api/ble_cmd.h"
#include "rf-core/api/common_cmd.h"

#include <stdio.h>
#include <string.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static unsigned char ble_params_buf[32];

int ble_nc_advertisement(void)
{
	uint32_t cmd_status;
	rfc_CMD_BLE_ADV_NC_t cmd;
	rfc_bleAdvPar_t *params;

	uint8_t channel = 37;
	char name[] = "test payload - msp@7.12.15";
	uint8_t *adv_payload = (uint8_t *) name;
	uint8_t adv_payload_len = strlen(name);

	PRINTF("ble_nc_advertisement called\n");

	params = (rfc_bleAdvPar_t *)ble_params_buf;

	/* Clear both buffers */
	memset(&cmd, 0x00, sizeof(cmd));
	memset(ble_params_buf, 0x00, sizeof(ble_params_buf));

	/* Adv NC */
	cmd.commandNo = CMD_BLE_ADV_NC;
	cmd.condition.rule = COND_NEVER;
	cmd.whitening.bOverride = 0;
	cmd.whitening.init = 0;
	cmd.pParams = params;
	cmd.channel = channel;

	/* Set up BLE Advertisement parameters */
	params->pDeviceAddress = (uint16_t *) &linkaddr_node_addr.u8[LINKADDR_SIZE - 2];
	params->endTrigger.triggerType = TRIG_NEVER;
	params->endTime = TRIG_NEVER;

	/* Set up BLE Advertisement parameters */
	params = (rfc_bleAdvPar_t *) ble_params_buf;
	params->advLen = adv_payload_len;
	params->pAdvData = adv_payload;

	if (rf_core_send_cmd((uint32_t) & cmd, &cmd_status) == RF_CORE_CMD_ERROR) {
		PRINTF("send_ble_adv_nc: Chan=%d CMDSTA=0x%08lx, status=0x%04x\n",
				channel, cmd_status, cmd.status);
		return RF_CORE_CMD_ERROR;
	}

	/* Wait until the command is done */
	if (rf_core_wait_cmd_done(&cmd) != RF_CORE_CMD_OK) {
		PRINTF("send_ble_adv_nc: Chan=%d CMDSTA=0x%08lx, status=0x%04x\n",
				channel, cmd_status, cmd.status);
		return RF_CORE_CMD_ERROR;
	}

	PRINTF("ble_nc_advertisement finished\n");

	return RF_CORE_CMD_OK;
}
