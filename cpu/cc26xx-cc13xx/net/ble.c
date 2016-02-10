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
#include <stdint.h>
#include <string.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define BLE_PARAMS_BUFFER_LENGTH 32

static unsigned char ble_params_buf[BLE_PARAMS_BUFFER_LENGTH];

int ble_send_command(rfc_bleRadioOp_t *cmd)
{
	uint32_t cmd_status;
	if (rf_core_send_cmd((uint32_t) cmd, &cmd_status) == RF_CORE_CMD_ERROR) {
		PRINTF("ble_send_advertisement: Chan=%d CMDSTA=0x%08lx, status=0x%04x\n",
				cmd->channel, cmd_status, cmd->status);
		return BLE_CMD_ERROR;
	}

	/* Wait until the command is done */
	if (rf_core_wait_cmd_done(cmd) != RF_CORE_CMD_OK) {
		PRINTF("ble_send_advertisement: Chan=%d CMDSTA=0x%08lx, status=0x%04x\n",
				cmd->channel, cmd_status, cmd->status);
		return BLE_CMD_ERROR;
	}
	return BLE_CMD_OK;
}

int ble_send_advertisement(int channel, char *payload, int payload_len)
{

	rfc_CMD_BLE_ADV_NC_t cmd;
	rfc_bleAdvPar_t *params;

	if(payload_len >= BLE_PARAMS_BUFFER_LENGTH || payload_len < 0) {
		PRINTF("ble_send_advertisement: payload length invalid\n");
		return BLE_CMD_ERROR;
	}

	params = (rfc_bleAdvPar_t *)ble_params_buf;

	/* Clear both buffers */
	memset(&cmd, 0x00, sizeof(cmd));
	memset(ble_params_buf, 0x00, sizeof(ble_params_buf));

	/* Adv NC */
	cmd.commandNo = CMD_BLE_ADV_NC;
//	cmd.commandNo = CMD_BLE_ADV_SCAN;
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
	params->advLen = payload_len;
	params->pAdvData = (uint8_t *) payload;

	int result = ble_send_command((rfc_bleRadioOp_t *) &cmd);

	if(result == BLE_CMD_OK) {
		PRINTF("ble_send_advertisement: successfully sent\n");
	}
	return result;
}
