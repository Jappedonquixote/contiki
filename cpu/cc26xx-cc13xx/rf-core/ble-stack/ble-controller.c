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
 * ble-controller.c
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "contiki.h"

#include "sys/process.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"

#include "dev/oscillators.h"

#include "rf-core/api/common_cmd.h"
#include "rf-core/api/data_entry.h"
#include "rf-core/api/mailbox.h"
#include "rf-core/api/ble_cmd.h"
#include "rf-core/api/common_cmd.h"
#include "rf-core/ble-stack/ble-addr.h"
#include "rf-core/ble-stack/ble-controller.h"
#include "rf-core/rf-core.h"

#include "net/packetbuf.h"

#define BLE_STATUS_CONN_REQ     5

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
/* advertising parameters */
/* use 10 seconds as default for the advertising interval */
static unsigned int adv_interval = (CLOCK_SECOND * 10);

/* use all 3 advertising channels per default */
static char adv_channel_map = 0b0111;

static unsigned int adv_data_len = 0;
static char* adv_data = NULL;
static unsigned int scan_resp_data_len = 0;
static char* scan_resp_data = NULL;

static short advertise = 0;
static struct etimer adv_timer;

/*---------------------------------------------------------------------------*/
/* connection parameters */
static unsigned long access_address = 0;
static unsigned short crc_init_0 = 0;
static unsigned short crc_init_1 = 0;
static unsigned short crc_init_2 = 0;
static unsigned short win_size = 0;
static unsigned int win_offset = 0;
static unsigned int interval = 0;
static unsigned int latency = 0;
static unsigned int timeout;
static unsigned long long channel_map = 0;
static unsigned short hops;
static unsigned short sca;

/*---------------------------------------------------------------------------*/
/* buffers for interacting with the radio controller */
static uint8_t ble_command_buf[BLE_COMMAND_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_params_buf[BLE_PARAMS_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_output_buf[BLE_OUTPUT_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_rx_buf_0[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_rx_buf_1[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_rx_buf_2[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_rx_buf_3[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);

/* The RX Data Queue */
static dataQueue_t rx_data_queue = { 0 };
/* Receive entry pointer to keep track of read items */
volatile static uint8_t *current_rx_entry;
/*---------------------------------------------------------------------------*/
void print_command_status(uint32_t cmd_status, uint16_t status_field)
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
/*---------------------------------------------------------------------------*/
unsigned short setup_ble_mode()
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
        PRINTF("setup_ble_mode() send: ");
        print_command_status(cmd_status, cmd.status);
        return BLE_COMMAND_ERROR;
    }

    /* Wait until radio setup is done */
    if(rf_core_wait_cmd_done(&cmd) != RF_CORE_CMD_OK) {
        PRINTF("setup_ble_mode() wait: ");
        print_command_status(cmd_status, cmd.status);
        return BLE_COMMAND_ERROR;
    }

    return BLE_COMMAND_SUCCESS;
}
/*---------------------------------------------------------------------------*/
PROCESS(ble_controller_advertising_process, "BLE/CC26xx advertising process");
/*---------------------------------------------------------------------------*/
unsigned short ble_controller_is_enabled()
{
    return rf_core_is_accessible();
}

/*---------------------------------------------------------------------------*/
unsigned short ble_controller_reset()
{
    PRINTF("ble_controller_reset()\n");
    rf_core_set_modesel();
    process_start(&rf_core_process, NULL);
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
void setup_buffers(void)
{
    /* clear all buffers */
    memset(ble_params_buf, 0x00, sizeof(ble_params_buf));
    memset(ble_rx_buf_0, 0x00, sizeof(ble_rx_buf_0));
    memset(ble_rx_buf_1, 0x00, sizeof(ble_rx_buf_1));
    memset(ble_rx_buf_2, 0x00, sizeof(ble_rx_buf_2));
    memset(ble_rx_buf_3, 0x00, sizeof(ble_rx_buf_3));
    memset(ble_output_buf, 0x00, sizeof(ble_output_buf));

    /* setup circular receive buffer queue (last entry = NULL) */
    rx_data_queue.pCurrEntry = ble_rx_buf_0;
    rx_data_queue.pLastEntry = NULL;
    current_rx_entry = ble_rx_buf_0;

    rfc_dataEntry_t *entry;
    entry = (rfc_dataEntry_t *)ble_rx_buf_0;
    entry->pNextEntry = ble_rx_buf_1;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_rx_buf_0) - 8;

    entry = (rfc_dataEntry_t *)ble_rx_buf_1;
    entry->pNextEntry = ble_rx_buf_2;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_rx_buf_1) - 8;

    entry = (rfc_dataEntry_t *)ble_rx_buf_2;
    entry->pNextEntry = ble_rx_buf_3;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_rx_buf_2) - 8;

    entry = (rfc_dataEntry_t *)ble_rx_buf_3;
    entry->pNextEntry = ble_rx_buf_0;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_rx_buf_3) - 8;
}

/*---------------------------------------------------------------------------*/
unsigned short ble_controller_enable()
{
    PRINTF("ble_controller_enable()\n");

    setup_buffers();

    oscillators_request_hf_xosc();

    if(rf_core_is_accessible())
    {
        return BLE_COMMAND_SUCCESS;
    }

    if(rf_core_boot() != RF_CORE_CMD_OK)
    {
        PRINTF("ble_controller_enable(): could not boot rf-core\n");
        return BLE_COMMAND_ERROR;
    }

    rf_core_setup_interrupts();
    oscillators_switch_to_hf_xosc();

    if(setup_ble_mode() != BLE_COMMAND_SUCCESS)
    {
        PRINTF("ble_controller_enable(): could not setup ble mode of rf-core\n");
        return BLE_COMMAND_ERROR;
    }

    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
unsigned short ble_controller_disable()
{
    PRINTF("ble_controller_disable()\n");

    rf_core_power_down();
    oscillators_switch_to_hf_rc();
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
unsigned short ble_controller_set_advertising_parameters(
        unsigned int advertising_interval, char advertising_channel_map)
{
    if((advertising_channel_map < BLE_ADV_CHANNEL_MASK_MIN) ||
       (advertising_channel_map > BLE_ADV_CHANNEL_MASK_MAX))
    {
        PRINTF("ble_controller_set_advertising_parameters(): invalid advertising channel mask\n");
        return BLE_COMMAND_ERROR;
    }
    adv_interval = advertising_interval;
    adv_channel_map = advertising_channel_map;
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
unsigned short ble_controller_set_advertising_data(
        unsigned int advertising_data_length, char* advertising_data)
{
    if(advertising_data_length > BLE_ADV_DATA_LENGHT_MAX)
    {
        PRINTF("ble_controller_set_advertising_data(): invalid advertising data length\n");
        return BLE_COMMAND_ERROR;
    }
    adv_data_len = advertising_data_length;
    adv_data = advertising_data;
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
unsigned short ble_controller_set_scan_response_data(
        unsigned int scan_response_data_length, char* scan_response_data)
{
    if(scan_response_data_length > BLE_SCAN_RESP_DATA_LENGHT_MAX)
    {
        PRINTF("ble_controller_set_scan_response_data(): invalid scan response data length\n");
        return BLE_COMMAND_ERROR;
    }
    scan_resp_data_len = scan_response_data_length;
    scan_resp_data = scan_response_data;
    return BLE_COMMAND_SUCCESS;
}

unsigned short send_advertisement(unsigned short channel)
{
    char result;
    uint32_t cmd_status;
    rfc_CMD_BLE_ADV_t *cmd;
    rfc_bleAdvPar_t *params;
    rfc_bleAdvOutput_t *output;

    cmd = (rfc_CMD_BLE_ADV_t *) ble_command_buf;
    params = (rfc_bleAdvPar_t *) ble_params_buf;
    output = (rfc_bleAdvOutput_t *) ble_output_buf;

    /* clear all used buffers */
    memset(ble_command_buf, 0x00, sizeof(ble_command_buf));
    memset(ble_params_buf, 0x00, sizeof(ble_params_buf));

    /* construct the command */
    cmd->commandNo = CMD_BLE_ADV;
    cmd->condition.rule = COND_NEVER;
    cmd->whitening.bOverride = 0;
    cmd->channel = channel;
    cmd->pParams = params;
    cmd->startTrigger.triggerType = TRIG_NOW;
    cmd->pOutput = output;

    /* construct the command parameters */
    params->pRxQ = &rx_data_queue;
    params->rxConfig.bAutoFlushIgnored = 0;
    params->rxConfig.bAutoFlushCrcErr = 1;
    params->rxConfig.bAutoFlushEmpty = 0;
    params->rxConfig.bIncludeLenByte = 1;
    params->rxConfig.bIncludeCrc = 0;
    params->rxConfig.bAppendRssi = 1;
    params->rxConfig.bAppendStatus = 1;
    params->rxConfig.bAppendTimestamp = 0;
    params->advConfig.advFilterPolicy = 0;
    params->advConfig.deviceAddrType = 0;
    params->advConfig.bStrictLenFilter = 0;
    params->advLen = adv_data_len;
    params->scanRspLen = scan_resp_data_len;
    params->pAdvData = (uint8_t *) adv_data;
    params->pScanRspData = (uint8_t *) scan_resp_data;
    params->pDeviceAddress = (uint16_t *) ble_node_addr.addr;
    params->endTrigger.triggerType = TRIG_NEVER;

    /* Send Radio setup to RF Core */
    if(rf_core_send_cmd((uint32_t)cmd, &cmd_status) != RF_CORE_CMD_OK) {
        PRINTF("send_advertisement() send: ");
        print_command_status(cmd_status, cmd->status);
        return BLE_COMMAND_ERROR;
    }

    /* Wait until radio setup is done */
    if(rf_core_wait_cmd_done(cmd) != RF_CORE_CMD_OK) {
        PRINTF("send_advertisement() wait: ");
        print_command_status(cmd_status, cmd->status);
        return BLE_COMMAND_ERROR;
    }

    return result;
}

/*---------------------------------------------------------------------------*/
unsigned short ble_controller_enable_advertising(void)
{
    advertise = 1;
    process_start(&ble_controller_advertising_process, NULL);
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
unsigned short ble_controller_disable_advertising(void)
{
    advertise = 0;
    process_poll(&ble_controller_advertising_process);
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
unsigned short get_current_rx_entry_pdu_type(void)
{
    rfc_dataEntryGeneral_t *entry = (rfc_dataEntryGeneral_t *) current_rx_entry;

    if(entry->status != DATA_ENTRY_FINISHED)
    {
        return 0;
    }

    return (current_rx_entry[9] & 0x0F);   // the header is in the 9th element of the receive buffer
}

/*---------------------------------------------------------------------------*/
unsigned short parse_connect_request_data()
{
    int data_offset = 23;
    rfc_dataEntryGeneral_t *entry = (rfc_dataEntryGeneral_t *) current_rx_entry;
    if(entry->status != DATA_ENTRY_FINISHED)
    {
        return BLE_COMMAND_ERROR;
    }

//    memcpy(&access_address, &current_rx_entry[data_offset], 4);
    access_address = (current_rx_entry[data_offset] << 24) + (current_rx_entry[data_offset + 1] << 16) +
                     (current_rx_entry[data_offset + 2] << 8) + current_rx_entry[data_offset + 3];



    crc_init_0 = current_rx_entry[data_offset + 4];
    crc_init_1 = current_rx_entry[data_offset + 5];
    crc_init_2 = current_rx_entry[data_offset + 6];
    win_size = current_rx_entry[data_offset + 7];
    win_offset = (current_rx_entry[data_offset + 9] << 8) + current_rx_entry[data_offset + 8];
    interval = (current_rx_entry[data_offset + 11] << 8) + current_rx_entry[data_offset + 10];
    latency = (current_rx_entry[data_offset + 13] << 8) + current_rx_entry[data_offset + 12];
    timeout = (current_rx_entry[data_offset + 15] << 8) + current_rx_entry[data_offset + 14];
    memcpy(&channel_map, &current_rx_entry[data_offset + 16], 5);
    hops = (current_rx_entry[data_offset + 21] & 0x01F);
    sca = (current_rx_entry[data_offset + 21] >> 5) & 0x07;

    PRINTF("parse_connect_request_data()\n");
    PRINTF("access address:   %20lX\n", access_address);
    PRINTF("crc_init_0:       %20X\n", crc_init_0);
    PRINTF("crc_init_1:       %20X\n", crc_init_1);
    PRINTF("crc_init_2:       %20X\n", crc_init_2);
    PRINTF("win_size:         %20X\n", win_size);
    PRINTF("win_offset:       %20X\n", win_offset);
    PRINTF("interval:         %20X\n", interval);
    PRINTF("latency:          %20X\n", latency);
    PRINTF("timeout:          %20X\n", timeout);
    PRINTF("channel_map:      %20llX\n", channel_map);
    PRINTF("hops:             %20X\n", hops);
    PRINTF("sca:              %20X\n", sca);
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
unsigned short ble_controller_read_current_rx_buf(
        void *buffer, unsigned short buffer_length)
{
    int len = 0;
    int8_t rssi;
    uint8_t channel;
    rfc_dataEntryGeneral_t *entry = (rfc_dataEntryGeneral_t *) current_rx_entry;

    if(entry->status != DATA_ENTRY_FINISHED)
    {
        return 0;
    }

    len = current_rx_entry[8] - 2;              // last 2 bytes are status and rssi
    memcpy(buffer, (char *)&current_rx_entry[9], len);

    rssi = (int8_t) current_rx_entry[8 + len + 1];
    channel = (uint8_t)current_rx_entry[8 + len + 2];
    packetbuf_set_attr(PACKETBUF_ATTR_CHANNEL, channel);
    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rssi);


    return len;
}

/*---------------------------------------------------------------------------*/
void ble_controller_free_current_rx_buf()
{
    rfc_dataEntryGeneral_t *entry = (rfc_dataEntryGeneral_t *)current_rx_entry;

    /* clear the length*/
    current_rx_entry[8] = 0;

    /* set status to pending */
    entry->status = DATA_ENTRY_PENDING;
    current_rx_entry = entry->pNextEntry;
}

/*---------------------------------------------------------------------------*/
unsigned short send_slave_command()
{
    char result;
    uint32_t cmd_status;
    rfc_CMD_BLE_SLAVE_t *cmd;
    rfc_bleSlavePar_t *params;
    rfc_bleMasterSlaveOutput_t *output;

    cmd = (rfc_CMD_BLE_SLAVE_t *) ble_command_buf;
    params = (rfc_bleSlavePar_t *) ble_params_buf;
    output = (rfc_bleMasterSlaveOutput_t *) ble_output_buf;

    /* clear all used buffers */
    memset(ble_command_buf, 0x00, sizeof(ble_command_buf));
    memset(ble_params_buf, 0x00, sizeof(ble_params_buf));
    memset(ble_output_buf, 0x00, sizeof(ble_output_buf));

    /* construct the command */
    cmd->commandNo = CMD_BLE_SLAVE;
    cmd->condition.rule = COND_NEVER;
    cmd->whitening.bOverride = 0;
    cmd->channel = hops;
    cmd->pParams = params;
    cmd->startTrigger.triggerType = TRIG_NOW;
    cmd->pOutput = output;

    /* construct the command parameters */
    params->pRxQ = &rx_data_queue;
//    params->pTxQ = 0;
    params->rxConfig.bAutoFlushIgnored = 0;
    params->rxConfig.bAutoFlushCrcErr = 0;
    params->rxConfig.bAutoFlushEmpty = 0;
    params->rxConfig.bIncludeLenByte = 1;
    params->rxConfig.bIncludeCrc = 0;
    params->rxConfig.bAppendRssi = 1;
    params->rxConfig.bAppendStatus = 1;
    params->rxConfig.bAppendTimestamp = 0;
    params->maxNack = 0;
    params->maxPkt = 0;
    params->accessAddress = access_address;
    params->crcInit0 = crc_init_0;
    params->crcInit1 = crc_init_1;
    params->crcInit2 = crc_init_2;
    params->timeoutTrigger.triggerType = TRIG_NEVER;
    params->endTrigger.triggerType = TRIG_NEVER;

    /* Send Radio setup to RF Core */
    if(rf_core_send_cmd((uint32_t)cmd, &cmd_status) != RF_CORE_CMD_OK) {
        PRINTF("send_slave_command() send: ");
        print_command_status(cmd_status, cmd->status);
        return BLE_COMMAND_ERROR;
    }

    /* Wait until radio setup is done */
    if(rf_core_wait_cmd_done(cmd) != RF_CORE_CMD_OK) {
        PRINTF("send_slave_command() wait: ");
        print_command_status(cmd_status, cmd->status);
        return BLE_COMMAND_ERROR;
    }

    return result;
}

static void print_status()
{
    rfc_bleRadioOp_t *cmd = (rfc_bleRadioOp_t *) ble_command_buf;

    PRINTF("BLE command status\n");
    PRINTF("number:  %X\n", cmd->commandNo);
    PRINTF("channel: %d\n", cmd->channel);
    print_command_status(0x00, cmd->status);
}

static void print_slave_output()
{
    rfc_bleMasterSlaveOutput_t *output = (rfc_bleMasterSlaveOutput_t *) ble_output_buf;

    PRINTF("BLE slave command output\n");
    PRINTF("nTx:           %d\n", output->nTx);
    PRINTF("nTxAck:        %d\n", output->nTxAck);
    PRINTF("nTxCtrl:       %d\n", output->nTxCtrl);
    PRINTF("nTxCtrlAck:    %d\n", output->nTxCtrlAck);
    PRINTF("nTxCtrlAckAck: %d\n", output->nTxCtrlAckAck);
    PRINTF("nTxRetrans:    %d\n", output->nTxRetrans);
    PRINTF("nTxEntryDone:  %d\n", output->nTxEntryDone);
    PRINTF("nRxOk:         %d\n", output->nRxOk);
    PRINTF("nRxCtrl:       %d\n", output->nRxCtrl);
    PRINTF("nRxCtlrAck:    %d\n", output->nRxCtrlAck);
    PRINTF("nRxNok:        %d\n", output->nRxNok);
    PRINTF("nRxIgnored:    %d\n", output->nRxIgnored);
    PRINTF("nRxEmpty:      %d\n", output->nRxEmpty);
    PRINTF("nRxBufFull:    %d\n", output->nRxBufFull);
    PRINTF("lastRssi:      %d\n", output->lastRssi);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_controller_advertising_process, ev, data)
{
    PROCESS_BEGIN();
    PRINTF("ble_controller_advertising_process: started\n");
    while(advertise == 1)
    {
        if(adv_channel_map & BLE_ADV_CHANNEL_1_MASK) {
            send_advertisement(BLE_ADV_CHANNEL_1);
        }
        if(adv_channel_map & BLE_ADV_CHANNEL_2_MASK) {
            send_advertisement(BLE_ADV_CHANNEL_2);
        }
        if(adv_channel_map & BLE_ADV_CHANNEL_3_MASK) {
            send_advertisement(BLE_ADV_CHANNEL_3);
        }
        etimer_set(&adv_timer, adv_interval);

        PROCESS_YIELD_UNTIL(etimer_expired(&adv_timer) ||
                            ev == PROCESS_EVENT_POLL ||
                            ev == rf_core_event);

        if(ev == rf_core_event && get_current_rx_entry_pdu_type() == BLE_STATUS_CONN_REQ) {
           PRINTF("ble_controller_connection_process: connection request received\n");
           if(parse_connect_request_data() != BLE_COMMAND_SUCCESS)
           {
               PRINTF("ble_controller_advertising_process: could not parse header\n");
           }
           else
           {
               ble_controller_disable_advertising();
               send_slave_command();
               etimer_set(&adv_timer, adv_interval);
               PROCESS_YIELD_UNTIL(etimer_expired(&adv_timer));
               print_status();
               print_slave_output();
           }
        }

    }
    PRINTF("ble_controller_advertising_process: advertising stopped\n");
    PROCESS_END();
}
