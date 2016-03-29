/*
 * Copyright (c) 2016, Michael Spoerk
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
 * rf-core-debug.c
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "rf-core-debug.h"
#include "api/common_cmd.h"
#include "api/data_entry.h"
#include "api/mailbox.h"
#include "api/ble_cmd.h"
#include "api/common_cmd.h"
#include "ble-stack/ble-addr.h"
#include "ble-stack/ble-controller.h"
#include "rf-core.h"
#include "net/packetbuf.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
#define RESOLVE_STATUS_BYTES 1

/*---------------------------------------------------------------------------*/
void print_cmdsta(uint32_t cmdsta)
{
#if RESOLVE_STATUS_BYTES
    PRINTF("CMDSTA: ");
    switch(cmdsta & RF_CORE_CMDSTA_RESULT_MASK)
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
    PRINTF(" (0x%08lx)\n", cmdsta);
#else
    PRINTF("CMDSTA: 0x%08lx\n", cmdsta);
#endif
}
/*---------------------------------------------------------------------------*/
void print_command_status(uint16_t status_field)
{
#if RESOLVE_STATUS_BYTES
    PRINTF("cmd->status: ");
    switch(status_field)
    {
      case RF_CORE_RADIO_OP_STATUS_IDLE:                  PRINTF("IDLE"); break;
      case RF_CORE_RADIO_OP_STATUS_PENDING:               PRINTF("PENDING"); break;
      case RF_CORE_RADIO_OP_STATUS_ACTIVE:                PRINTF("ACTIVE"); break;
      case RF_CORE_RADIO_OP_STATUS_ERROR_PAST_START:      PRINTF("ERROR_PAST_START"); break;
      case RF_CORE_RADIO_OP_STATUS_ERROR_START_TRIG:      PRINTF("ERROR_START_TRIG"); break;
      case RF_CORE_RADIO_OP_STATUS_ERROR_CONDITION:       PRINTF("ERROR_CONDITION"); break;
      case RF_CORE_RADIO_OP_STATUS_ERROR_PAR:             PRINTF("ERROR_PAR"); break;
      case RF_CORE_RADIO_OP_STATUS_ERROR_POINTER:         PRINTF("ERROR_POINTER"); break;
      case RF_CORE_RADIO_OP_STATUS_ERROR_CMDID:           PRINTF("ERROR_CMDID"); break;
      case RF_CORE_RADIO_OP_STATUS_ERROR_NO_SETUP:        PRINTF("ERROR_NO_SETUP"); break;
      case RF_CORE_RADIO_OP_STATUS_ERROR_NO_FS:           PRINTF("ERROR_NO_FS"); break;
      case RF_CORE_RADIO_OP_STATUS_ERROR_SYNTH_PROG:      PRINTF("ERROR_SYNTH_PROG"); break;
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


/*---------------------------------------------------------------------------*/
void print_slave_output(uint8_t *ble_slave_output_buf)
{
    rfc_bleMasterSlaveOutput_t *output = (rfc_bleMasterSlaveOutput_t *) ble_slave_output_buf;

    PRINTF("slave output\n");
    PRINTF("nTx:                            %16d\n", output->nTx);
    PRINTF("nTxAck:                         %16d\n", output->nTxAck);
    PRINTF("nRxOk:                          %16d\n", output->nRxOk);
    PRINTF("nRxNok:                         %16d\n", output->nRxNok);
    PRINTF("nRxCtrl:                        %16d\n", output->nRxCtrl);
    PRINTF("nRxCtrlAck:                     %16d\n", output->nRxCtrlAck);
    PRINTF("nRxIgn:                         %16d\n", output->nRxIgnored);
    PRINTF("nRxEmpty:                       %16d\n", output->nRxEmpty);
    PRINTF("nRxBufFull:                     %16d\n", output->nRxBufFull);
    PRINTF("pktStatus.lastAck:              %16d\n", output->pktStatus.bLastAck);
    PRINTF("pktStatus.bLastCrcErr:          %16d\n", output->pktStatus.bLastCrcErr);
    PRINTF("pktStatus.bLastCtrl:            %16d\n", output->pktStatus.bLastCtrl);
    PRINTF("pktStatus.bLastEmpty:           %16d\n", output->pktStatus.bLastEmpty);
    PRINTF("pktStatus.bLastIgnored:         %16d\n", output->pktStatus.bLastIgnored);
    PRINTF("pktStatus.bLastMd:              %16d\n", output->pktStatus.bLastMd);
    PRINTF("pktStatus.bTimeStampValid:      %16d\n", output->pktStatus.bTimeStampValid);
    PRINTF("timestamp:                      %16lu\n", output->timeStamp);
}

/*---------------------------------------------------------------------------*/
void print_slave_sequence_stats(uint8_t *ble_slave_params_buf)
{
    rfc_bleSlavePar_t *params = (rfc_bleSlavePar_t *) ble_slave_params_buf;
    PRINTF("seqStats.bAutoEmpty         %d\n", params->seqStat.bAutoEmpty);
    PRINTF("seqStats.bFirstPkt          %d\n", params->seqStat.bFirstPkt);
    PRINTF("seqStats.bLlCtrlAckPending  %d\n", params->seqStat.bLlCtrlAckPending);
    PRINTF("seqStats.bLlCtrlAckRx       %d\n", params->seqStat.bLlCtrlAckRx);
    PRINTF("seqStats.bLlCtrlTx          %d\n", params->seqStat.bLlCtrlTx);
    PRINTF("seqStats.lastRxSn           %d\n", params->seqStat.lastRxSn);
    PRINTF("seqStats.lastTxSn           %d\n", params->seqStat.lastTxSn);
    PRINTF("seqStats.nextTxSn           %d\n", params->seqStat.nextTxSn);
}
