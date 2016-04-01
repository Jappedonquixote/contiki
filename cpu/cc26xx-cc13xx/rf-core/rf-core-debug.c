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
    uint8_t val = (uint8_t) (cmdsta & RF_CORE_CMDSTA_RESULT_MASK);

    if(val == RF_CORE_CMDSTA_PENDING) {
        PRINTF("CMDSTA: PENDING\n");
    }
    else if(val == RF_CORE_CMDSTA_DONE) {
        PRINTF("CMDSTA: DONE\n");
    }
    else if(val == RF_CORE_CMDSTA_ILLEGAL_PTR) {
        PRINTF("CMDSTA: ILLEGAL_PTR\n");
    }
    else if(val == RF_CORE_CMDSTA_UNKNOWN_CMD) {
        PRINTF("CMDSTA: UNKNOWN_CMD\n");
    }
    else if(val == RF_CORE_CMDSTA_UNKNOWN_DIR_CMD) {
        PRINTF("CMDSTA: UNKNOWN_DIR_CMD\n");
    }
    else if(val == RF_CORE_CMDSTA_CONTEXT_ERR) {
        PRINTF("CMDSTA: CONTEXT_ERR\n");
    }
    else if(val == RF_CORE_CMDSTA_SCHEDULING_ERR) {
        PRINTF("CMDSTA: SCHEDULING_ERR\n");
    }
    else if(val == RF_CORE_CMDSTA_PAR_ERR) {
        PRINTF("CMDSTA: PAR_ERR\n");
    }
    else if(val == RF_CORE_CMDSTA_QUEUE_ERR) {
        PRINTF("CMDSTA: QUEUE_ERR\n");
    }
    else if(val == RF_CORE_CMDSTA_QUEUE_BUSY) {
        PRINTF("CMDSTA: QUEUE_BUSY\n");
    }
    else {
        PRINTF("CMDSTA: 0x%08lx\n", cmdsta);
    }
}
/*---------------------------------------------------------------------------*/
void print_command_status(uint16_t status_field)
{

    if (status_field == RF_CORE_RADIO_OP_STATUS_IDLE)
    {
        PRINTF("cmd->status: IDLE\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_PENDING)
    {
        PRINTF("cmd->status: PENDING\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_ACTIVE)
    {
        PRINTF("cmd->status: ACTIVE\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_ERROR_PAST_START)
    {
        PRINTF("cmd->status: ERROR_PAST_START\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_ERROR_START_TRIG)
    {
        PRINTF("cmd->status: ERROR_START_TRIG\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_ERROR_CONDITION)
    {
        PRINTF("cmd->status: ERROR_CONDITION\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_ERROR_PAR)
    {
        PRINTF("cmd->status: ERROR_PAR\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_ERROR_POINTER)
    {
        PRINTF("cmd->status: ERROR_POINTER\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_ERROR_CMDID)
    {
        PRINTF("cmd->status: ERROR_CMDID\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_ERROR_NO_SETUP)
    {
        PRINTF("cmd->status: ERROR_NO_SETUP\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_ERROR_NO_FS)
    {
        PRINTF("cmd->status: ERROR_NO_FS\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_ERROR_SYNTH_PROG)
    {
        PRINTF("cmd->status: ERROR_SYNTH_PROG\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_DONE_OK)
    {
        PRINTF("cmd->status: BLE_DONE_OK\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_DONE_RXTIMEOUT)
    {
        PRINTF("cmd->status: BLE_DONE_RXTIMEOUT\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_DONE_NOSYNC)
    {
        PRINTF("cmd->status: BLE_DONE_NOSYNC\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_DONE_RXERR)
    {
        PRINTF("cmd->status: BLE_DONE_RXERR\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_DONE_CONNECT)
    {
        PRINTF("cmd->status: BLE_DONE_CONNECT\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_DONE_MAXNACK)
    {
        PRINTF("cmd->status: BLE_DONE_MAXNACK\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_DONE_ENDED)
    {
        PRINTF("cmd->status: BLE_DONE_ENDED\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_DONE_ABORT)
    {
        PRINTF("cmd->status: BLE_DONE_ABORT\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_DONE_STOPPED)
    {
        PRINTF("cmd->status: BLE_DONE_STOPPED\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_ERROR_PAR)
    {
        PRINTF("cmd->status: BLE_ERROR_PAR\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_ERROR_RXBUF)
    {
        PRINTF("cmd->status: BLE_ERROR_RXBUF\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_ERROR_NO_SETUP)
    {
        PRINTF("cmd->status: BLE_ERROR_NO_SETUP\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_ERROR_NO_FS)
    {
        PRINTF("cmd->status: BLE_ERROR_NO_FS\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_ERROR_SYNTH_PROG)
    {
        PRINTF("cmd->status: BLE_ERROR_SYNTH_PROT\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_ERROR_RXOVF)
    {
        PRINTF("cmd->status: BLE_ERROR_RXOVF\n");
    }
    else if (status_field == RF_CORE_RADIO_OP_STATUS_BLE_ERROR_TXUNF)
    {
        PRINTF("cmd->status: BLE_ERROR_TXUNF\n");
    }
    else
    {
        PRINTF("cmd->status: 0x%04X\n", status_field);
    }
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

