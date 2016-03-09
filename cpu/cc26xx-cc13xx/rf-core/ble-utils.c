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
 * ble-radio-interface.c
 *
 *  Created on: 09. März 2016
 *      Author: Michael Spörk
 */
/*---------------------------------------------------------------------------*/
#include "rf-core/ble-utils.h"
#include "contiki.h"
#include "dev/radio.h"
#include "dev/cc26xx-uart.h"
#include "dev/oscillators.h"

#include "lpm.h"

#include "ti-lib.h"
#include "net/linkaddr.h"
#include "rf-core/rf-core.h"
#include "rf-core/api/data_entry.h"
#include "rf-core/api/mailbox.h"
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

#define RESOLVE_STATUS_BYTES 1

#define MAX_ADVERTISEMENT_DATA_LENGTH       31
#define MAX_SCAN_RESPONSE_DATA_LENGTH       31

#define BLE_PARAMS_BUFFER_LENGTH  32
#define BLE_RECEIVE_BUFFER_LENGTH 48
#define BLE_OUTPUT_BUFFER_LENGTH  16

static uint8_t ble_params_buf[BLE_PARAMS_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_receive_buf[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_output_buf[BLE_OUTPUT_BUFFER_LENGTH] CC_ALIGN(4);

/* The RX Data Queue */
static dataQueue_t rx_data_queue = { 0 };

void print_cmd_status(uint32_t cmd_status, uint16_t status_field)
{
#if RESOLVE_STATUS_BYTES
    PRINTF("CMDSTA: ");
    switch(cmd_status & RF_CORE_CMDSTA_RESULT_MASK)
    {
      case RF_CORE_CMDSTA_PENDING:          PRINTF("PENDING"); break;
      case RF_CORE_CMDSTA_DONE:             PRINTF("DONE"); break;
      case RF_CORE_CMDSTA_ILLEGAL_PTR:      PRINTF("ILLEGAL_PTR"); break;
      case RF_CORE_CMDSTA_UNKNOWN_CMD:      PRINTF("UNKNOWN_CMD"); break;
      case RF_CORE_CMDSTA_UNKNOWN_DIR_CMD:  PRINTF("UNKWOWN_DIR_CMD"); break;
      case RF_CORE_CMDSTA_CONTEXT_ERR:      PRINTF("CONTEXT_ERR"); break;
      case RF_CORE_CMDSTA_SCHEDULING_ERR:   PRINTF("SCHEDULING_ERR"); break;
      case RF_CORE_CMDSTA_PAR_ERR:          PRINTF("PAR_ERR"); break;
      case RF_CORE_CMDSTA_QUEUE_ERR:        PRINTF("QUEUE_ERR"); break;
      case RF_CORE_CMDSTA_QUEUE_BUSY:       PRINTF("QUEUE_BUSY"); break;
      default:                              PRINTF("unknown");
    }
    PRINTF(" (0x%08lx) status: ", cmd_status);

    switch(status_field)
    {
      case RF_CORE_RADIO_OP_STATUS_IDLE:                  PRINTF("IDLE"); break;
      case RF_CORE_RADIO_OP_STATUS_PENDING:               PRINTF("PENDING"); break;
      case RF_CORE_RADIO_OP_STATUS_ACTIVE:                PRINTF("ACTIVE"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_DONE_OK:           PRINTF("BLE_DONE_OK"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_DONE_RXTIMEOUT:    PRINTF("BLE_DONE_RXTIMEOUT"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_DONE_NOSYNC:       PRINTF("BLE_DONE_NOSYNC"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_DONE_RXERR:        PRINTF("BLE_DONE_RXERR"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_DONE_CONNECT:      PRINTF("BLE_DONE_CONNECT"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_DONE_MAXNACK:      PRINTF("BLE_DONE_MAXNACK"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_DONE_ENDED:        PRINTF("BLE_DONE_ENDED"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_DONE_ABORT:        PRINTF("BLE_DONE_ABORT"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_DONE_STOPPED:      PRINTF("BLE_DONE_STOPPED"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_ERROR_PAR:         PRINTF("BLE_ERROR_PAR"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_ERROR_RXBUF:       PRINTF("BLE_ERROR_RXBUF"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_ERROR_NO_SETUP:    PRINTF("BLE_ERROR_NO_SETUP"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_ERROR_NO_FS:       PRINTF("BLE_ERROR_NO_FS"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_ERROR_SYNTH_PROG:  PRINTF("BLE_ERROR_SYNTH_PROT"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_ERROR_RXOVF:       PRINTF("BLE_ERROR_RXOVF"); break;
      case RF_CORE_RADIO_OP_STATUS_BLE_ERROR_TXUNF:       PRINTF("BLE_ERROR_TXUNF"); break;
      default:                                            PRINTF("unknown");
    }
    PRINTF(" (0x%04x)\n", status_field);
#else
    PRINTF("CMDSTA: 0x%08lx, status: 0x%04x\n", cmd_status, status_field);
#endif
}

uint8_t ble_send_command(rfc_bleRadioOp_t *cmd)
{
  uint32_t cmd_status = 0;
  if (rf_core_send_cmd((uint32_t) cmd, &cmd_status) == RF_CORE_CMD_ERROR) {
    print_cmd_status(cmd_status, cmd->status);
    return BLE_CMD_ERROR;
  }
  /* Wait until the command is done */
  if (rf_core_wait_cmd_done(cmd) != RF_CORE_CMD_OK) {
      print_cmd_status(cmd_status, cmd->status);
    return BLE_CMD_ERROR;
  }
  print_cmd_status(cmd_status, cmd->status);
  return BLE_CMD_OK;
}

uint8_t ble_utils_send_advertisement(int channel,
                                     char *advertisement_data, int advertisement_data_len,
                                     char *scan_response_data, int scan_response_data_len)
{
    uint8_t result;
    rfc_CMD_BLE_ADV_t cmd;
    rfc_bleAdvPar_t *params;
    rfc_bleAdvOutput_t *output;

    if((channel < 37) || (channel > 39)) {
        PRINTF("ble_utils_send_advertisement: not a valid advertisement channel\n");
        return BLE_CMD_ERROR;
    }
    if((advertisement_data_len < 0) ||
       (advertisement_data_len > MAX_ADVERTISEMENT_DATA_LENGTH)) {
        PRINTF("ble_utils_send_advertisement: invalid advertisement data length\n");
        return BLE_CMD_ERROR;
    }
    if((scan_response_data_len < 0) ||
       (scan_response_data_len > MAX_SCAN_RESPONSE_DATA_LENGTH)) {
        PRINTF("ble_utils_send_advertisement: invalid scan response data length\n");
        return BLE_CMD_ERROR;
    }

    params = (rfc_bleAdvPar_t *) ble_params_buf;
    output = (rfc_bleAdvOutput_t *) ble_output_buf;

    /* clear all used buffers */
    memset(&cmd, 0x00, sizeof(cmd));
    memset(ble_params_buf, 0x00, sizeof(ble_params_buf));
    memset(ble_receive_buf, 0x00, sizeof(ble_receive_buf));

    /* construct the command */
    cmd.commandNo = CMD_BLE_ADV;
    cmd.condition.rule = COND_NEVER;
    cmd.whitening.bOverride = 0;
    cmd.channel = channel;
    cmd.pParams = params;
    cmd.startTrigger.triggerType = TRIG_NOW;
    cmd.pOutput = output;

    /* init the receive buffer */
    rfc_dataEntry_t *entry = (rfc_dataEntry_t *) ble_receive_buf;
    entry->pNextEntry = NULL;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_receive_buf) - 8;
    rx_data_queue.pCurrEntry = ble_receive_buf;
    rx_data_queue.pLastEntry = ble_receive_buf;

    /* construct the command parameters */
    params->pRxQ = &rx_data_queue;
    params->advConfig.advFilterPolicy = 0;
    params->advConfig.deviceAddrType = 0;
    params->advConfig.bStrictLenFilter = 0;
    params->advLen = advertisement_data_len;
    params->scanRspLen = scan_response_data_len;
    params->pAdvData = (uint8_t *) advertisement_data;
    params->pScanRspData = (uint8_t *) scan_response_data;
    params->pDeviceAddress = (uint16_t *) &linkaddr_node_addr.u8[LINKADDR_SIZE - 2];
    params->endTrigger.triggerType = TRIG_NEVER;

    result = ble_send_command((rfc_bleRadioOp_t *) &cmd);
    if(result == BLE_CMD_OK) {
        PRINTF("ble_utils_send_advertisement: successfully sent\n");
    }
    else {
        PRINTF("ble_utils_send_advertisement: could not send advertisement\n");
    }

    return result;
}

void ble_utils_print_rx_data()
{
    uint8_t loop;
    PRINTF("RX buffer data:\n");
    for(loop = 0; loop < BLE_RECEIVE_BUFFER_LENGTH; ++loop) {
        PRINTF("%02X ", (char) ble_receive_buf[loop]);
    }
    PRINTF("\n");
}

void ble_utils_print_advertisement_output_data()
{
    rfc_bleAdvOutput_t *output = (rfc_bleAdvOutput_t *) ble_output_buf;

    PRINTF("advertisement output data:\n");
    PRINTF("nTxAdvInd:  %d\n", output->nTxAdvInd);
    PRINTF("nTxScanRsp: %d\n", output->nTxScanRsp);
    PRINTF("nRxScanReq: %d\n", output->nRxScanReq);
    PRINTF("nRxConReq:  %d\n", output->nRxConnectReq);
    PRINTF("nRxNok:     %d\n", output->nRxNok);
    PRINTF("nRxIgnored: %d\n", output->nRxIgnored);
    PRINTF("nRxBufFull: %d\n", output->nRxBufFull);
    PRINTF("lastRssi:   %d\n", output->lastRssi);
    PRINTF("timeStamp:  %lu\n", output->timeStamp);
}
