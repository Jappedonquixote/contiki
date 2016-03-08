/*
 * Copyright (c) 2016, Michael Spörk
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/*
 * ble.c
 *
 *  Created on: 07. Dez. 2015
 *      Author: Michael Spörk <m.spoerk@student.tugraz.at>
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
#define BLE_OUTPUT_BUFFER_LENGTH 32

static unsigned char ble_params_buf[BLE_PARAMS_BUFFER_LENGTH];
static unsigned char ble_output_buf[BLE_OUTPUT_BUFFER_LENGTH];

int ble_send_command(rfc_bleRadioOp_t *cmd)
{
	uint32_t cmd_status;
	if (rf_core_send_cmd((uint32_t) cmd, &cmd_status) == RF_CORE_CMD_ERROR) {
		PRINTF("ble_send_command: Chan=%d CMDSTA=0x%08lx, status=0x%04x\n",
				cmd->channel, cmd_status, cmd->status);
		return BLE_CMD_ERROR;
	}

	/* Wait until the command is done */
	if (rf_core_wait_cmd_done(cmd) != RF_CORE_CMD_OK) {
		PRINTF("ble_send_command: Chan=%d CMDSTA=0x%08lx, status=0x%04x\n",
				cmd->channel, cmd_status, cmd->status);
		return BLE_CMD_ERROR;
	}
	return BLE_CMD_OK;
}

int ble_send_advertisement(int channel, char *payload, int payload_len)
{

	rfc_CMD_BLE_ADV_NC_t cmd;
	rfc_bleAdvPar_t *params;
	rfc_bleScannerOutput_t output;

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

int ble_start_scanner(int channel)
{
    rfc_CMD_BLE_SCANNER_t cmd;
    rfc_bleScannerPar_t *params;
    rfc_bleScannerOutput_t *output;

    printf("size of scanner parameter: %d\n", sizeof(rfc_bleScannerPar_t));
    printf("size of scanner output:    %d\n", sizeof(rfc_bleScannerOutput_t));

    params = (rfc_bleScannerPar_t *)ble_params_buf;
    output = (rfc_bleScannerOutput_t *)ble_output_buf;

    /* Clear all buffers */
    memset(&cmd, 0x00, sizeof(cmd));
    memset(ble_params_buf, 0x00, sizeof(ble_params_buf));
    memset(ble_output_buf, 0x00, sizeof(ble_output_buf));

    /* initialize command */
    cmd.commandNo = CMD_BLE_SCANNER;
    cmd.condition.rule = COND_NEVER;
    cmd.whitening.bOverride = 0;
    cmd.whitening.init = 0;
    cmd.pParams = params;
    cmd.pOutput = output;
    cmd.channel = channel;

    /* initialize parameters */
    params->scanConfig.bActiveScan = 0;     /* passive scanning */
    params->scanConfig.deviceAddrType = 0;  /* public device addresses */
    params->scanConfig.bStrictLenFilter = 0;/* accept messages with illegal length */
    params->scanConfig.bEndOnRpt = 0;       /* do not end scanning automatically */
    params->pDeviceAddress = (uint16_t *) &linkaddr_node_addr.u8[LINKADDR_SIZE - 2];
    params->endTrigger.triggerType = TRIG_NEVER;
    params->endTime = TRIG_NEVER;

    int result = ble_send_command((rfc_bleRadioOp_t *) &cmd);

    if(result == BLE_CMD_OK) {
      PRINTF("ble_start_scanner: successfully sent\n");
    }
    return result;
}

int ble_get_last_rssi()
{
  rfc_bleScannerOutput_t *output;
  output = (rfc_bleScannerOutput_t *)ble_output_buf;

  output->lastRssi;
}
