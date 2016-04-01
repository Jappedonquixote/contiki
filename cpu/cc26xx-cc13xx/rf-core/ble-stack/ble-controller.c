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
#include "rf-core/rf-core-debug.h"
#include "net/packetbuf.h"

#define RX_CONFIG_INCLUDE_STATUS    1
#define RX_CONFIG_INCLUDE_RSSI      1
#define RX_CONFIG_INCLUDE_TIMESTAMP 1

#define TIMESTAMP_LOCATION 0x40043004

#define STATUS_DONE                 RF_CORE_RADIO_OP_STATUS_BLE_DONE_OK
#define STATUS_CONN_REQ             RF_CORE_RADIO_OP_STATUS_BLE_DONE_CONNECT

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
/* BLE controller states */
typedef enum  {
    OFF,
    ADVERTISING,
    CONNECTED
} ble_controller_states_t;

static ble_controller_states_t state = OFF;
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

static struct etimer adv_timer;

/* advertising command buffers */
static uint8_t ble_adv_cmd_buf[BLE_COMMAND_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_adv_params_buf[BLE_PARAMS_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_adv_output_buf[BLE_OUTPUT_BUFFER_LENGTH] CC_ALIGN(4);
/*---------------------------------------------------------------------------*/
/* connection parameters */
/* all time values are stored in rf core ticks (4 MHz) */
static uint32_t access_address = 0;
static uint8_t crc_init_0 = 0;
static uint8_t crc_init_1 = 0;
static uint8_t crc_init_2 = 0;
static uint32_t win_size = 0;
static uint32_t win_offset = 0;
static uint32_t interval = 0;
static uint16_t latency = 0;
static uint32_t timeout;
static uint64_t data_channel_map = 0;
static uint8_t hops = 0;
static uint8_t sca = 0;
static uint32_t conn_req_timestamp = 0;
static uint32_t next_anchor_rf_ticks = 0;
static uint8_t current_channel = 0;

/* slave command buffers */
static uint8_t ble_slave_cmd_buf[BLE_COMMAND_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_slave_params_buf[BLE_PARAMS_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_slave_output_buf[BLE_OUTPUT_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_slave_tx_buf[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);

static dataQueue_t slave_tx_data_queue = { 0 };
volatile static uint8_t *slave_current_tx_entry;

static uint32_t conn_event_number = 0;

/*---------------------------------------------------------------------------*/
/* RX data queue (all received packets are stored in the same queue) */
static uint8_t rx_buf_0[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t rx_buf_1[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t rx_buf_2[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t rx_buf_3[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);

static dataQueue_t rx_data_queue = { 0 };
volatile static uint8_t *current_rx_entry;
/*---------------------------------------------------------------------------*/
/* Returns the status byte of a command buffer */
#define CMD_GET_STATUS(a) (((rfc_radioOp_t *)a)->status)
/*---------------------------------------------------------------------------*/
/* some time parameters in a BLE connection request need to be multiplied
 * by 1.25 to get the actual time in milliseconds
 * (see Bluetooth Specification v4)*/
#define CONN_PARAM_TO_TICKS(a) ((uint32_t) ((a) * 5000))
/* the supervision timeout in a BLE connection request need to be multiplied
 * by 10 to get the actual time in milliseconds
 * (see Bluetooth Specification v4)*/
#define CONN_PARAM_TIMEOUT_TO_TICKS(a) ((uint8_t) ((a) * 40000))
/*---------------------------------------------------------------------------*/
/* Converts ms into rf_core ticks  */
#define CONVERT_MS_TO_RF_CORE_TICKS(a) ((a) * 4000)
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
        print_cmdsta(cmd_status);
        print_command_status(cmd.status);
        return BLE_COMMAND_ERROR;
    }

    /* Wait until radio setup is done */
    if(rf_core_wait_cmd_done(&cmd) != RF_CORE_CMD_OK) {
        PRINTF("setup_ble_mode() wait: ");
        print_command_status(cmd.status);
        return BLE_COMMAND_ERROR;
    }

    return BLE_COMMAND_SUCCESS;
}
/*---------------------------------------------------------------------------*/
PROCESS(ble_controller_advertising_process, "BLE/CC26xx advertising process");
PROCESS(ble_controller_connection_process, "BLE/CC26xx connection process");
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
    /* ADVERTISING */
    memset(ble_adv_params_buf, 0x00, sizeof(ble_adv_params_buf));
    memset(ble_adv_output_buf, 0x00, sizeof(ble_adv_output_buf));

    rfc_dataEntry_t *entry;

    /* SLAVE */
    memset(ble_slave_params_buf, 0x00, sizeof(ble_slave_params_buf));
    memset(ble_slave_output_buf, 0x00, sizeof(ble_slave_output_buf));
    memset(ble_slave_tx_buf, 0x00, sizeof(ble_slave_tx_buf));

    /* slave transmit buffer */
    slave_tx_data_queue.pCurrEntry = ble_slave_tx_buf;
    slave_tx_data_queue.pLastEntry = ble_slave_tx_buf;
    slave_current_tx_entry = ble_slave_tx_buf;

    entry = (rfc_dataEntry_t *) ble_slave_tx_buf;
    entry->pNextEntry = NULL;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_slave_tx_buf) - 8;

    /* RX buffer */
    memset(rx_buf_0, 0x00, sizeof(rx_buf_0));
    memset(rx_buf_1, 0x00, sizeof(rx_buf_1));
    memset(rx_buf_2, 0x00, sizeof(rx_buf_2));
    memset(rx_buf_3, 0x00, sizeof(rx_buf_3));

    /* setup circular receive buffer queue (last entry = NULL) */
    rx_data_queue.pCurrEntry = rx_buf_0;
    rx_data_queue.pLastEntry = NULL;
    current_rx_entry = rx_buf_0;

    entry = (rfc_dataEntry_t *)rx_buf_0;
    entry->pNextEntry = rx_buf_1;
    entry->config.lenSz = 1;
    entry->length = sizeof(rx_buf_0) - 8;

    entry = (rfc_dataEntry_t *)rx_buf_1;
    entry->pNextEntry = rx_buf_2;
    entry->config.lenSz = 1;
    entry->length = sizeof(rx_buf_1) - 8;

    entry = (rfc_dataEntry_t *)rx_buf_2;
    entry->pNextEntry = rx_buf_3;
    entry->config.lenSz = 1;
    entry->length = sizeof(rx_buf_2) - 8;

    entry = (rfc_dataEntry_t *)rx_buf_3;
    entry->pNextEntry = rx_buf_0;
    entry->config.lenSz = 1;
    entry->length = sizeof(rx_buf_3) - 8;
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

/*---------------------------------------------------------------------------*/
void create_radio_cmd_now(rfc_bleRadioOp_t *cmd, uint8_t cmd_len,
                          uint16_t command_nr, uint8_t channel,
                          uint8_t *params, uint8_t *output)
{
    memset(cmd, 0x00, cmd_len);
    cmd->commandNo = command_nr;
    cmd->condition.rule = COND_NEVER;
    cmd->whitening.bOverride = 0;
    cmd->channel = channel;
    cmd->pParams = params;
    cmd->startTrigger.triggerType = TRIG_NOW;
    cmd->pOutput = output;
}

/*---------------------------------------------------------------------------*/
void create_radio_cmd_timestamp(rfc_bleRadioOp_t *cmd, uint8_t cmd_len,
                                uint16_t command_nr, uint8_t channel,
                                uint8_t *params, uint8_t *output,
                                uint32_t start_time)
{
    memset(cmd, 0x00, cmd_len);
    cmd->commandNo = command_nr;
    cmd->condition.rule = COND_NEVER;
    cmd->whitening.bOverride = 0;
    cmd->channel = channel;
    cmd->pParams = params;
    cmd->startTrigger.triggerType = TRIG_ABSTIME;
//    cmd->startTrigger.pastTrig = 1;
    cmd->startTrigger.pastTrig = 0;
    cmd->startTime = start_time;
    cmd->pOutput = output;
}

/*---------------------------------------------------------------------------*/
void create_adv_params(rfc_bleAdvPar_t *params)
{
    memset(params, 0x00, sizeof(BLE_PARAMS_BUFFER_LENGTH));
    params->pRxQ = &rx_data_queue;
    params->rxConfig.bAutoFlushIgnored = 0;
    params->rxConfig.bAutoFlushCrcErr = 1;
    params->rxConfig.bAutoFlushEmpty = 0;
    params->rxConfig.bIncludeLenByte = 1;
    params->rxConfig.bIncludeCrc = 0;
    params->rxConfig.bAppendRssi = RX_CONFIG_INCLUDE_RSSI;
    params->rxConfig.bAppendStatus = RX_CONFIG_INCLUDE_STATUS;
    params->rxConfig.bAppendTimestamp = RX_CONFIG_INCLUDE_TIMESTAMP;
    params->advConfig.advFilterPolicy = 0;
    params->advConfig.deviceAddrType = 0;
    params->advConfig.bStrictLenFilter = 0;
    params->advLen = adv_data_len;
    params->scanRspLen = scan_resp_data_len;
    params->pAdvData = (uint8_t *) adv_data;
    params->pScanRspData = (uint8_t *) scan_resp_data;
    params->pDeviceAddress = (uint16_t *) ble_node_addr.addr;
    params->endTrigger.triggerType = TRIG_NEVER;
}

/*---------------------------------------------------------------------------*/
unsigned short send_advertisement(rfc_bleRadioOp_t * cmd)
{
    uint32_t cmd_status;
    if(rf_core_send_cmd((uint32_t)cmd, &cmd_status) != RF_CORE_CMD_OK) {
        PRINTF("send_advertisement() could not send cmd to rf-core: \n");
        print_cmdsta(cmd_status);
        print_command_status(cmd->status);
        return BLE_COMMAND_ERROR;
    }
    if(rf_core_wait_cmd_done(cmd) != RF_CORE_CMD_OK) {
        PRINTF("send_advertisement() could not wait for cmd\n");
        print_command_status(cmd->status);
    }
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
unsigned short ble_controller_enable_advertising(void)
{
    state = ADVERTISING;
    process_start(&ble_controller_advertising_process, NULL);
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
unsigned short ble_controller_disable_advertising(void)
{
    state = OFF;
    process_poll(&ble_controller_advertising_process);
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_controller_advertising_process, ev, data)
{
    rfc_bleRadioOp_t *cmd = (rfc_bleRadioOp_t *) ble_adv_cmd_buf;
    rfc_bleAdvPar_t *params = (rfc_bleAdvPar_t *) ble_adv_params_buf;
    rfc_bleAdvOutput_t *output = (rfc_bleAdvOutput_t *) ble_adv_output_buf;

    PROCESS_BEGIN();
    PRINTF("ble_controller_advertising_process: started\n");
    while(state == ADVERTISING)
    {
        create_adv_params(params);
        /* advertise on the configured channels */
        if((cmd->status != RF_CORE_RADIO_OP_STATUS_BLE_DONE_CONNECT) &&
                (adv_channel_map & BLE_ADV_CHANNEL_1_MASK)) {
            create_radio_cmd_now(cmd, BLE_COMMAND_BUFFER_LENGTH, CMD_BLE_ADV,
                    BLE_ADV_CHANNEL_1, (uint8_t *) params, (uint8_t *) output);
            send_advertisement(cmd);
        }
        if((cmd->status != RF_CORE_RADIO_OP_STATUS_BLE_DONE_CONNECT) &&
                (adv_channel_map & BLE_ADV_CHANNEL_2_MASK)) {
            create_radio_cmd_now(cmd, BLE_COMMAND_BUFFER_LENGTH, CMD_BLE_ADV,
                    BLE_ADV_CHANNEL_2, (uint8_t *) params, (uint8_t *) output);
            send_advertisement(cmd);
        }
        if((cmd->status != RF_CORE_RADIO_OP_STATUS_BLE_DONE_CONNECT) &&
                (adv_channel_map & BLE_ADV_CHANNEL_3_MASK)) {
            create_radio_cmd_now(cmd, BLE_COMMAND_BUFFER_LENGTH, CMD_BLE_ADV,
                    BLE_ADV_CHANNEL_3, (uint8_t *) params, (uint8_t *) output);
            send_advertisement(cmd);
        }

        etimer_set(&adv_timer, adv_interval);
        PROCESS_YIELD_UNTIL(etimer_expired(&adv_timer) ||
                            ev == PROCESS_EVENT_POLL ||
                            ev == rf_core_data_event);

        /* some data was received by the rf core */
        if((ev == rf_core_data_event)) {
            if(CMD_GET_STATUS(ble_adv_cmd_buf) == STATUS_CONN_REQ)
            {
                /* received data was connection request
                 * so quit advertising and start connection process */
                state = OFF;
                process_start(&ble_controller_connection_process, NULL);
            }
            else {
                /* received data was scan request or something other than
                 * a connection request -> keep advertising */
                PROCESS_YIELD_UNTIL(etimer_expired(&adv_timer)
                                    ||  ev == PROCESS_EVENT_POLL);
            }
        }
    }
    PRINTF("ble_controller_advertising_process: advertising stopped\n");
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void print_data_queue_entry(uint8_t *entry)
{
    int i = 0;
    /* length of the ble data entry */
    /* decrease the length of ble data by rssi, status and timestamp, if included */
    int len = MAX(entry[8] - (1 * RX_CONFIG_INCLUDE_RSSI) -
            (1 * RX_CONFIG_INCLUDE_STATUS) - (4 * RX_CONFIG_INCLUDE_TIMESTAMP), 0);
    int ble_offset = 9;
    rfc_dataEntryGeneral_t *e = (rfc_dataEntryGeneral_t *) entry;

    PRINTF("RX buffer data entry\n");
    PRINTF("status: ");

    if(e->status == DATA_ENTRY_PENDING) {
        PRINTF("status: PENDING\n");
    }
    else if(e->status == DATA_ENTRY_ACTIVE) {
        PRINTF("status: ACTIVE\n");
    }
    else if(e->status == DATA_ENTRY_BUSY) {
        PRINTF("status: BUSY\n");
    }
    else if(e->status == DATA_ENTRY_FINISHED) {
        PRINTF("status: FINISHED\n");
    }
    else if(e->status == DATA_ENTRY_UNFINISHED) {
        PRINTF("status: UNFINISHED\n");
    }
    else {
        PRINTF("status: unknown\n");
    }
    PRINTF("size: %d\n", len);
    PRINTF("BLE packet data:");
    for(i = ble_offset; i < ble_offset + len; ++i)
    {
        PRINTF("0x%02X ", entry[i]);
    }

    if(RX_CONFIG_INCLUDE_RSSI)
    {
        PRINTF("\nRSSI: %d", ((int8_t)entry[i++]));
    }
    if(RX_CONFIG_INCLUDE_STATUS)
    {
        PRINTF("\nstatus: %d", entry[i++]);
    }
    PRINTF("\n");
}
/*---------------------------------------------------------------------------*/
unsigned short ble_controller_read_current_rx_buf(
        void *buffer, unsigned short buffer_length)
{
    int len = 0;
    int8_t rssi;
    uint8_t channel;
    rfc_dataEntryGeneral_t *entry;

    entry = (rfc_dataEntryGeneral_t *) current_rx_entry;
    if(entry->status != DATA_ENTRY_FINISHED) {
        return 0;
    }
    /* len is the length of ble data*/
    /* since the buffer also contains status, rssi and timestamp information
     * after the ble data, these 6 bytes need to be removed from the length calculation*/
    len = current_rx_entry[8] - 6;
    if(len > 0)
    {
        memcpy(buffer, (char *) &current_rx_entry[9], len);
        rssi = current_rx_entry[9 + len];
        channel = current_rx_entry[9 + len + 1];
        packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rssi);
        packetbuf_set_attr(PACKETBUF_ATTR_CHANNEL, channel);
    }
    return len;
}

/*---------------------------------------------------------------------------*/
void ble_controller_free_current_rx_buf()
{
    rfc_dataEntryGeneral_t *entry = (rfc_dataEntryGeneral_t *) current_rx_entry;
    /* clear the length field */
    current_rx_entry[8] = 0;
    /* set status to pending */
    entry->status = DATA_ENTRY_PENDING;
    /* set next data queue entry */
    current_rx_entry = entry->pNextEntry;
}

/*---------------------------------------------------------------------------*/
/* The parameter are parsed according to Bluetooth Specification v4 (page 2510)*/
unsigned short parse_connect_request_data()
{
    int data_offset = 23;
    memcpy(&access_address, &current_rx_entry[data_offset], 4);
    crc_init_0 = current_rx_entry[data_offset + 4];
    crc_init_1 = current_rx_entry[data_offset + 5];
    crc_init_2 = current_rx_entry[data_offset + 6];
//    win_size = current_rx_entry[data_offset + 7];
//    win_offset = (current_rx_entry[data_offset + 9] << 8) + current_rx_entry[data_offset + 8];
//    interval = (current_rx_entry[data_offset + 11] << 8) + current_rx_entry[data_offset + 10];
    win_size = CONN_PARAM_TO_TICKS(current_rx_entry[data_offset + 7]);
    win_offset = CONN_PARAM_TO_TICKS((current_rx_entry[data_offset + 9] << 8) + current_rx_entry[data_offset + 8]);
    interval = CONN_PARAM_TO_TICKS((current_rx_entry[data_offset + 11] << 8) + current_rx_entry[data_offset + 10]);
    latency = (current_rx_entry[data_offset + 13] << 8) + current_rx_entry[data_offset + 12];
    timeout = CONN_PARAM_TIMEOUT_TO_TICKS((current_rx_entry[data_offset + 15] << 8) + current_rx_entry[data_offset + 14]);
    memcpy(&data_channel_map, &current_rx_entry[data_offset + 16], 5);
    hops = (current_rx_entry[data_offset + 21] & 0x01F);
    sca = (current_rx_entry[data_offset + 21] >> 5) & 0x07;
    conn_req_timestamp = (current_rx_entry[data_offset + 27] << 24) + (current_rx_entry[data_offset + 26] << 16) +
            (current_rx_entry[data_offset + 25] << 8) + (current_rx_entry[data_offset + 24]);
    current_channel = hops;
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
void create_slave_params(rfc_bleSlavePar_t *params, uint8_t first_packet, uint8_t transmit_data)
{
    uint32_t timeout_time;
    if(first_packet) {
        timeout_time = win_size;
        memset(params, 0x00, sizeof(BLE_PARAMS_BUFFER_LENGTH));
    }
    else {
        timeout_time = interval / 2;
    }
    params->pRxQ = &rx_data_queue;

    if(transmit_data > 0) {
        // if slave command needs to transmit data, then add queue
        params->pTxQ = &slave_tx_data_queue;
    }
    else {
        // if no data needed, then send auto-acks
        params->pTxQ = NULL;
    }
    params->rxConfig.bAutoFlushIgnored = 0;
    params->rxConfig.bAutoFlushCrcErr = 0;
    params->rxConfig.bAutoFlushEmpty = 0;
    params->rxConfig.bIncludeLenByte = 1;
    params->rxConfig.bIncludeCrc = 0;
    params->rxConfig.bAppendRssi = RX_CONFIG_INCLUDE_RSSI;
    params->rxConfig.bAppendStatus = RX_CONFIG_INCLUDE_STATUS;
    params->rxConfig.bAppendTimestamp = RX_CONFIG_INCLUDE_TIMESTAMP;

    if(first_packet > 0) {
        // set parameters for first packet according to TI Technical Reference Manual
        params->seqStat.lastRxSn = 1;
        params->seqStat.lastTxSn = 1;
        params->seqStat.nextTxSn = 0;
        params->seqStat.bFirstPkt = 1;
        params->seqStat.bAutoEmpty = 0;
        params->seqStat.bLlCtrlTx = 0;
        params->seqStat.bLlCtrlAckRx = 0;
        params->seqStat.bLlCtrlAckPending = 0;
    }

    params->maxNack = 0;
    params->maxPkt = 0;
    params->accessAddress = access_address;
    params->crcInit0 = crc_init_0;
    params->crcInit1 = crc_init_1;
    params->crcInit2 = crc_init_2;
    params->timeoutTrigger.triggerType = TRIG_REL_START;
    params->timeoutTime = timeout_time;
    params->endTrigger.triggerType = TRIG_NEVER;
}

/*---------------------------------------------------------------------------*/
uint8_t send_slave_command(rfc_bleRadioOp_t * cmd)
{
    uint32_t cmd_status;
    if(rf_core_send_cmd((uint32_t)cmd, &cmd_status) != RF_CORE_CMD_OK) {
        PRINTF("send_slave_command() could not send cmd to rf-core: \n");
        print_cmdsta(cmd_status);
        print_command_status(cmd->status);
        return BLE_COMMAND_ERROR;
    }
    return BLE_COMMAND_SUCCESS;
}
/*---------------------------------------------------------------------------*/
void conn_event()
{
    rfc_bleRadioOp_t *cmd = (rfc_bleRadioOp_t *) ble_slave_cmd_buf;
    rfc_bleSlavePar_t *params = (rfc_bleSlavePar_t *) ble_slave_params_buf;
    rfc_bleMasterSlaveOutput_t *output = (rfc_bleMasterSlaveOutput_t *) ble_slave_output_buf;


    /* calculate the number of the upcoming connection event */
    conn_event_number++;

    if(conn_event_number >= 5)
    {
        return;
    }

    /* after successfully exchanging packets, the following conditions are met: */
     /* - the output.timestamp is valid and stores the anchor for all following connection events */
     if(output->pktStatus.bTimeStampValid != 1)
     {
         PRINTF("ble_controller_connection_process no timestamp available\n");
         return;
     }

    /* calculate the data channel of the upcoming connection event */
    current_channel = (current_channel + hops) % 37;
    /* calculate the anchor of the upcoming connection event */
    next_anchor_rf_ticks = output->timeStamp + (conn_event_number * (interval - 2) * 4000);

    create_slave_params(params, 0, 0);
    create_radio_cmd_timestamp(cmd, BLE_COMMAND_BUFFER_LENGTH, CMD_BLE_SLAVE,
                               current_channel, (uint8_t *) params, (uint8_t *) output,
                               next_anchor_rf_ticks);
    send_slave_command(cmd);
}
static uint32_t time_delta;

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_controller_connection_process, ev, data)
{
    rfc_bleRadioOp_t *cmd = (rfc_bleRadioOp_t *) ble_slave_cmd_buf;
    rfc_bleSlavePar_t *params = (rfc_bleSlavePar_t *) ble_slave_params_buf;
    rfc_bleMasterSlaveOutput_t *output = (rfc_bleMasterSlaveOutput_t *) ble_slave_output_buf;
    uint32_t next_wakeup;
    uint32_t current_time;

    PROCESS_BEGIN();
//    PRINTF("ble_controller_connection_process started\n");
    state = CONNECTED;
    parse_connect_request_data();

    /* this is an experiment to only send the first packet in the 2nd interval */
//    next_wakeup = conn_req_timestamp + win_offset + interval - 2000;
    current_channel = (current_channel + hops) % 37;
    next_wakeup = conn_req_timestamp + win_offset + interval * 2 - 2000;
    rf_core_start_timer_comp(next_wakeup);

    /* create and send the first slave command */
    create_slave_params(params, 1, 0);
    create_radio_cmd_timestamp(cmd, BLE_COMMAND_BUFFER_LENGTH, CMD_BLE_SLAVE,
                               current_channel, (uint8_t *) params, (uint8_t *) output,
//                               conn_req_timestamp + win_offset - 1000);
                               conn_req_timestamp + win_offset + interval - 2000);
    send_slave_command(cmd);

    time_delta = RTIMER_NOW();
//    PRINTF("ble_controller_connection_process slave command sent\n");
//
//    PRINTF("*************************************************************\n");
//    PRINTF("CONNECTION PARAMETER:\n");
//    PRINTF("access address:            0x%8lX\n", access_address);
//    PRINTF("crc init 0:                      0x%02X\n", crc_init_0);
//    PRINTF("crc init 1:                      0x%02X\n", crc_init_1);
//    PRINTF("crc init 2:                      0x%02X\n", crc_init_2);
//    PRINTF("win_size:            %16lu\n", win_size);
//    PRINTF("win_offset:          %16lu\n", win_offset);
    PRINTF("interval:            %16lu\n", interval);
//    PRINTF("latency:                           %02u\n", latency);
//    PRINTF("timeout:             %16lu\n", timeout);
//    PRINTF("hops:                              %02u\n", hops);
//    PRINTF("sca:                               %02u\n", sca);
//    PRINTF("current_channel:                   %02u\n", current_channel);

//    PRINTF("*************************************************************\n");
//    PRINTF("TIMING MEASUREMENTS:\n");
    PRINTF("conn_event_number:   %16lu\n", conn_event_number);
//    PRINTF("conn_req_timestamp:  %16lu\n", conn_req_timestamp);
//    PRINTF("expected start:      %16lu\n", (conn_req_timestamp + win_offset));
    PRINTF("next_wakeup:         %16lu\n", next_wakeup);

//    PRINTF("*************************************************************\n");
//    PRINTF("COMMAND STATUS/OUTPUT:\n");
//    print_command_status(CMD_GET_STATUS(ble_slave_cmd_buf));
//    print_slave_output(ble_slave_output_buf);
    time_delta = RTIMER_NOW() - time_delta;
    PRINTF("time_delta: %lu\n", time_delta);

//    PRINTF("*************************************************************\n");
//    PRINTF("RX QUEUE ENTRIES:\n");
//    PRINTF("buffer 0:\n");
//    print_data_queue_entry(rx_buf_0);
//    PRINTF("buffer 1:\n");
//    print_data_queue_entry(rx_buf_1);
//    PRINTF("buffer 2:\n");
//    print_data_queue_entry(rx_buf_2);
//    PRINTF("buffer 3:\n");
//    print_data_queue_entry(rx_buf_3);

    while(1)
    {
        PROCESS_WAIT_EVENT_UNTIL(ev == rf_core_timer_event);

        PRINTF("timer event\n");

        /* calculate the number of the upcoming connection event */
        conn_event_number++;
        /* calculate the data channel of the upcoming connection event */
        current_channel = (current_channel + hops) % 37;
        /* calculate the anchor of the upcoming connection event */
        next_anchor_rf_ticks = output->timeStamp + (conn_event_number * interval);

        create_slave_params(params, 0, 0);
        create_radio_cmd_timestamp(cmd, BLE_COMMAND_BUFFER_LENGTH, CMD_BLE_SLAVE,
                                   current_channel, (uint8_t *) params, (uint8_t *) output,
                                   next_anchor_rf_ticks);
        memcpy(&current_time, TIMESTAMP_LOCATION, 4);
        send_slave_command(cmd);




        PRINTF("*************************************************************\n");
        PRINTF("conn_event_number:   %16lu\n", conn_event_number);
        PRINTF("current_channel:                   %02u\n", current_channel);
        PRINTF("current_time:        %16lu\n", current_time);
        PRINTF("timestamp:           %16lu\n", output->timeStamp);
        PRINTF("next_anchor_ticks:   %16lu\n", next_anchor_rf_ticks);

        PRINTF("*************************************************************\n");
        PRINTF("COMMAND STATUS/OUTPUT:\n");
        print_command_status(CMD_GET_STATUS(ble_slave_cmd_buf));
        print_slave_output(ble_slave_output_buf);

        PRINTF("*************************************************************\n");
        PRINTF("RX QUEUE ENTRIES:\n");
        PRINTF("buffer 0:\n");
        print_data_queue_entry(rx_buf_0);
        PRINTF("buffer 1:\n");
        print_data_queue_entry(rx_buf_1);
        PRINTF("buffer 2:\n");
        print_data_queue_entry(rx_buf_2);
        PRINTF("buffer 3:\n");
        print_data_queue_entry(rx_buf_3);
    }
    PROCESS_END();
}
