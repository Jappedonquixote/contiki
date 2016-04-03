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

#include "ble-radio-advertising.h"

#include "sys/process.h"
#include "sys/rtimer.h"

#include "rf-core/api/data_entry.h"
#include "rf-core/api/ble_cmd.h"
#include "rf-core/ble-stack/ble-addr.h"
#include "rf-core/rf-core.h"
#include "rf-core/rf-core-debug.h"

#define ADV_PDU_CONN_REQ            5
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
/* advertising parameters */
static unsigned int adv_interval = BLE_ADV_INTERVAL;
static char adv_channel_map = BLE_ADV_CHANNEL_1_MASK;
static unsigned int adv_data_len = 0;
static char adv_data[BLE_ADV_DATA_LENGHT_MAX];
static unsigned int scan_resp_data_len = 0;
static char scan_resp_data[BLE_SCAN_RESP_DATA_LENGHT_MAX];

static struct rtimer adv_timer;
static bool advertising = 0;
/*---------------------------------------------------------------------------*/
/* advertising command buffers */
static rfc_CMD_BLE_ADV_t adv_cmd;
static rfc_bleAdvPar_t adv_param;
static rfc_bleAdvOutput_t adv_output;
/*---------------------------------------------------------------------------*/
/* RX data queue (all received packets are stored in the same queue) */
static uint8_t rx_buf_0[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t rx_buf_1[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t rx_buf_2[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t rx_buf_3[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);

static dataQueue_t rx_data_queue = { 0 };
volatile static uint8_t *current_rx_entry;

static uint8_t rx_adv_pdu_type;

static ble_conn_req_data_t conn_req_data;

/*---------------------------------------------------------------------------*/
void init_advertising_data()
{
    adv_data_len = 0;
    memset(adv_data, 0x00, BLE_ADV_DATA_LENGHT_MAX);
    /* BLE flags */
    adv_data[adv_data_len++] = 2;
    adv_data[adv_data_len++] = 0x01;
    adv_data[adv_data_len++] = 0x05;   /* LE limited discoverable (no BR/EDR support) */
    /* TX power level */
    adv_data[adv_data_len++] = 2;
    adv_data[adv_data_len++] = 0x0A;
    adv_data[adv_data_len++] = 0;      /* 0 dBm; TODO: get actual tx power value */
    /* service UUIDs (16-bit identifiers) */
    adv_data[adv_data_len++] = 3;
    adv_data[adv_data_len++] = 0x03;
    adv_data[adv_data_len++] = 0x20;
    adv_data[adv_data_len++] = 0x18;   /* only IP support service exposed */
    /* service UUIDs (32-bit identifiers) */
    adv_data[adv_data_len++] = 1;
    adv_data[adv_data_len++] = 0x05;   /* empty list */
    /* service UUIDs (128-bit identifiers) */
    adv_data[adv_data_len++] = 1;
    adv_data[adv_data_len++] = 0x07;   /* empty list */
}
/*---------------------------------------------------------------------------*/
void init_scan_resp_data()
{
    scan_resp_data_len = 0;
    memset(scan_resp_data, 0x00, BLE_ADV_DATA_LENGHT_MAX);
    /* complete device name */
    scan_resp_data[scan_resp_data_len++] = 1 + strlen(BLE_DEVICE_NAME);
    scan_resp_data[scan_resp_data_len++] = 0x09;
    memcpy(&scan_resp_data[scan_resp_data_len],
           BLE_DEVICE_NAME, strlen(BLE_DEVICE_NAME));
    scan_resp_data_len += strlen(BLE_DEVICE_NAME);
    /* slave connection interval range */
    scan_resp_data[scan_resp_data_len++] = 5;
    scan_resp_data[scan_resp_data_len++] = 0x12;
    scan_resp_data[scan_resp_data_len++] = (BLE_SLAVE_CONN_INTERVAL_MIN & 0xFF);
    scan_resp_data[scan_resp_data_len++] = ((BLE_SLAVE_CONN_INTERVAL_MIN >> 8) & 0xFF);
    scan_resp_data[scan_resp_data_len++] = (BLE_SLAVE_CONN_INTERVAL_MAX & 0xFF);
    scan_resp_data[scan_resp_data_len++] = ((BLE_SLAVE_CONN_INTERVAL_MAX >> 8) & 0xFF);
}

/*---------------------------------------------------------------------------*/
PROCESS(ble_radio_advertising_process, "BLE/CC26xx advertising process");
process_event_t conn_req_event;
/*---------------------------------------------------------------------------*/
void setup_adv_buffers(void)
{
    /* ADVERTISING */
    memset(&adv_cmd, 0x00, sizeof(adv_cmd));
    memset(&adv_param, 0x00, sizeof(adv_param));
    memset(&adv_output, 0x00, sizeof(adv_output));

    rfc_dataEntry_t *entry;

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
unsigned short ble_radio_advertising_set_advertising_parameters(
        unsigned int advertising_interval, char advertising_channel_map)
{
    if((advertising_channel_map < BLE_ADV_CHANNEL_MASK_MIN) ||
       (advertising_channel_map > BLE_ADV_CHANNEL_MASK_MAX))
    {
        PRINTF("ble_radio_advertising_set_advertising_parameters(): invalid advertising channel mask\n");
        return 0;
    }
    adv_interval = advertising_interval;
    adv_channel_map = advertising_channel_map;
    return 1;
}

///*---------------------------------------------------------------------------*/
//unsigned short ble_radio_advertising_set_advertising_data(
//        unsigned int advertising_data_length, char* advertising_data)
//{
//    if(advertising_data_length > BLE_ADV_DATA_LENGHT_MAX)
//    {
//        PRINTF("ble_radio_advertising_set_advertising_data(): invalid advertising data length\n");
//        return 0;
//    }
//    adv_data_len = advertising_data_length;
//    adv_data = advertising_data;
//    return 1;
//}
//
///*---------------------------------------------------------------------------*/
//unsigned short ble_radio_advertising_set_scan_response_data(
//        unsigned int scan_response_data_length, char* scan_response_data)
//{
//    if(scan_response_data_length > BLE_SCAN_RESP_DATA_LENGHT_MAX)
//    {
//        PRINTF("ble_radio_advertising_set_scan_response_data(): invalid scan response data length\n");
//        return 0;
//    }
//    scan_resp_data_len = scan_response_data_length;
//    scan_resp_data = scan_response_data;
//    return 1;
//}
/*---------------------------------------------------------------------------*/
void create_adv_cmd(rfc_CMD_BLE_ADV_t  *cmd, uint8_t channel,
                    rfc_bleAdvPar_t *params, rfc_bleAdvOutput_t *output)
{
    memset(cmd, 0x00, sizeof(rfc_CMD_BLE_ADV_t));
    cmd->commandNo = CMD_BLE_ADV;
    cmd->condition.rule = COND_NEVER;
    cmd->whitening.bOverride = 0;
    cmd->channel = channel;
    cmd->pParams = params;
    cmd->startTrigger.triggerType = TRIG_NOW;
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
    params->rxConfig.bAppendRssi = 0;
    params->rxConfig.bAppendStatus = 0;
    params->rxConfig.bAppendTimestamp = 1;
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
unsigned short send_advertisement(rfc_CMD_BLE_ADV_t * cmd)
{
    uint32_t cmd_status;
    if(rf_core_send_cmd((uint32_t)cmd, &cmd_status) != RF_CORE_CMD_OK) {
        PRINTF("send_advertisement() could not send cmd to rf-core: \n");
        print_cmdsta(cmd_status);
        print_command_status(cmd->status);
        return 0;
    }
    if(rf_core_wait_cmd_done(cmd) != RF_CORE_CMD_OK) {
        PRINTF("send_advertisement() could not wait for cmd\n");
        print_command_status(cmd->status);
    }
    return 1;
}

/*---------------------------------------------------------------------------*/
unsigned short ble_radio_advertising_enable_advertising(void)
{
    init_advertising_data();
    init_scan_resp_data();
    advertising = 1;
    process_start(&ble_radio_advertising_process, NULL);
    return 1;
}

/*---------------------------------------------------------------------------*/
unsigned short ble_radio_advertising_disable_advertising(void)
{
    advertising = 0;
    process_exit(&ble_radio_advertising_process);
    return 1;
}
/*---------------------------------------------------------------------------*/
void advertising_event(struct rtimer *timer, void *ptr)
{
    if(!advertising)
    {
        /* stop sending advertisements */
        return;
    }
    if(adv_channel_map & BLE_ADV_CHANNEL_1_MASK)
    {
        create_adv_cmd(&adv_cmd, BLE_ADV_CHANNEL_1, &adv_param, &adv_output);
        send_advertisement(&adv_cmd);
    }
    if(adv_channel_map & BLE_ADV_CHANNEL_2_MASK)
    {
        create_adv_cmd(&adv_cmd, BLE_ADV_CHANNEL_2, &adv_param, &adv_output);
        send_advertisement(&adv_cmd);
    }
    if(adv_channel_map & BLE_ADV_CHANNEL_3_MASK)
    {
        create_adv_cmd(&adv_cmd, BLE_ADV_CHANNEL_3, &adv_param, &adv_output);
        send_advertisement(&adv_cmd);
    }
    rtimer_set(&adv_timer, RTIMER_NOW() + (adv_interval * RTIMER_SECOND),
               0, advertising_event, NULL);
}

/*---------------------------------------------------------------------------*/
/* The parameter are parsed according to Bluetooth Specification v4 (page 2510)*/
void parse_connect_request_data(void)
{
    int offset = 23;
    uint8_t *entry = (uint8_t *) current_rx_entry;

    memcpy(&conn_req_data.access_address, &entry[offset], 4);
    conn_req_data.crc_init_0 = entry[offset + 4];
    conn_req_data.crc_init_1 = entry[offset + 5];
    conn_req_data.crc_init_2 = entry[offset + 6];
    conn_req_data.win_size = entry[offset + 7];
    conn_req_data.win_offset = (entry[offset + 9] << 8) + entry[offset + 8];
    conn_req_data.interval = (entry[offset + 11] << 8) + entry[offset + 10];
    conn_req_data.latency = (entry[offset + 13] << 8) + entry[offset + 12];
    conn_req_data.timeout = (entry[offset + 15] << 8) + entry[offset + 14];
    memcpy(&conn_req_data.channel_map, &entry[offset + 16], 5);
    conn_req_data.hop = entry[offset + 21] & 0x1F;
    conn_req_data.sca = (entry[offset + 21] >> 5) & 0x07;
    conn_req_data.timestamp = (entry[offset + 25] << 24) +
                              (entry[offset + 24] << 16) +
                              (entry[offset + 23] << 8) +
                               entry[offset + 22];
}

/*---------------------------------------------------------------------------*/
void process_current_rx_buf(void)
{
    int pdu_offset = 9;                     // start index of BLE data

    /* reset current advertising PDU type */
    rx_adv_pdu_type = 0;

    rfc_dataEntryGeneral_t *entry = (rfc_dataEntryGeneral_t *) current_rx_entry;
    if(entry->status != DATA_ENTRY_FINISHED)
    {
        return;
    }

    /* read ADV_PDU type of the current rx entry */
    rx_adv_pdu_type = (current_rx_entry[pdu_offset] & 0x0F);

    /* if ADV_PDU is of type CONN_REQ -> parse conn request data */
    if(rx_adv_pdu_type == ADV_PDU_CONN_REQ)
    {
        parse_connect_request_data();
    }

    /* free current rx entry */
    /* clear the length field */
    current_rx_entry[8] = 0;
    /* set status to pending */
    entry->status = DATA_ENTRY_PENDING;
    /* set next data queue entry */
    current_rx_entry = entry->pNextEntry;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_radio_advertising_process, ev, data)
{
    PROCESS_BEGIN();

    conn_req_event = process_alloc_event();

    PRINTF("ble_radio_advertising_process() started\n");

    /* initialization */
    setup_adv_buffers();
    create_adv_params(&adv_param);
    advertising_event(&adv_timer, NULL);

    while(advertising)
    {
        PROCESS_WAIT_EVENT_UNTIL(ev == rf_core_data_event);
        process_current_rx_buf();

        if(rx_adv_pdu_type == ADV_PDU_CONN_REQ)
        {
            /* CONN REQ received */
            advertising = 0;
            process_post(PROCESS_BROADCAST, conn_req_event,
                         (process_data_t) &conn_req_data);
            PRINTF("ble_radio_advertising_process() conn_req\n");
        }
    }
    PRINTF("ble_radio_advertising_process() stopped\n");
    PROCESS_END();
}
