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


#define RF_RADIO_OP_GET_STATUS(a) (((rfc_radioOp_t *)a)->status)

#define BLE_STATUS_CONN_REQ     5

/* used for converting ticks of the rf_core into rtimer seconds */
#define RF_CORE_TICKS_IN_RTIMER_SEC(a) (((uint32_t) (a)/61))

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
static uint8_t ble_adv_rx_buf_0[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_adv_rx_buf_1[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_adv_rx_buf_2[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_adv_rx_buf_3[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);

/* The RX Data Queue */
static dataQueue_t adv_rx_data_queue = { 0 };
/* Receive entry pointer to keep track of read items */
volatile static uint8_t *adv_current_rx_entry;

/*---------------------------------------------------------------------------*/
/* connection parameters */
static uint32_t access_address = 0;
static uint8_t crc_init_0 = 0;
static uint8_t crc_init_1 = 0;
static uint8_t crc_init_2 = 0;
static uint8_t win_size = 0;
static uint16_t win_offset = 0;
static uint16_t interval = 0;
static uint16_t latency = 0;
static uint16_t timeout;
static uint64_t data_channel_map = 0;
static uint8_t hops = 0;
static uint8_t sca = 0;
static uint32_t conn_req_timestamp_rf_ticks = 0;
static uint32_t next_anchor_rf_ticks = 0;
static uint8_t current_channel = 0;

static struct rtimer conn_timer;

/* slave command buffers */
static uint8_t ble_slave_cmd_buf[BLE_COMMAND_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_slave_params_buf[BLE_PARAMS_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_slave_output_buf[BLE_OUTPUT_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_slave_rx_buf_0[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_slave_rx_buf_1[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);
static uint8_t ble_slave_tx_buf[BLE_RECEIVE_BUFFER_LENGTH] CC_ALIGN(4);

static dataQueue_t slave_rx_data_queue = { 0 };
volatile static uint8_t *slave_current_rx_entry;

static dataQueue_t slave_tx_data_queue = { 0 };
volatile static uint8_t *slave_current_tx_entry;

static uint32_t conn_event_number = 0;
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
    /* clear all buffers */
    memset(ble_adv_params_buf, 0x00, sizeof(ble_adv_params_buf));
    memset(ble_adv_rx_buf_0, 0x00, sizeof(ble_adv_rx_buf_0));
    memset(ble_adv_rx_buf_1, 0x00, sizeof(ble_adv_rx_buf_1));
    memset(ble_adv_rx_buf_2, 0x00, sizeof(ble_adv_rx_buf_2));
    memset(ble_adv_rx_buf_3, 0x00, sizeof(ble_adv_rx_buf_3));
    memset(ble_adv_output_buf, 0x00, sizeof(ble_adv_output_buf));

    /* setup circular receive buffer queue (last entry = NULL) */
    adv_rx_data_queue.pCurrEntry = ble_adv_rx_buf_0;
    adv_rx_data_queue.pLastEntry = NULL;
    adv_current_rx_entry = ble_adv_rx_buf_0;

    rfc_dataEntry_t *entry;
    entry = (rfc_dataEntry_t *)ble_adv_rx_buf_0;
    entry->pNextEntry = ble_adv_rx_buf_1;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_adv_rx_buf_0) - 8;

    entry = (rfc_dataEntry_t *)ble_adv_rx_buf_1;
    entry->pNextEntry = ble_adv_rx_buf_2;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_adv_rx_buf_1) - 8;

    entry = (rfc_dataEntry_t *)ble_adv_rx_buf_2;
    entry->pNextEntry = ble_adv_rx_buf_3;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_adv_rx_buf_2) - 8;

    entry = (rfc_dataEntry_t *)ble_adv_rx_buf_3;
    entry->pNextEntry = ble_adv_rx_buf_0;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_adv_rx_buf_3) - 8;

    /* SLAVE */
    memset(ble_slave_params_buf, 0x00, sizeof(ble_slave_params_buf));
    memset(ble_slave_rx_buf_0, 0x00, sizeof(ble_slave_rx_buf_0));
    memset(ble_slave_rx_buf_1, 0x00, sizeof(ble_slave_rx_buf_1));
    memset(ble_slave_output_buf, 0x00, sizeof(ble_slave_output_buf));
    memset(ble_slave_tx_buf, 0x00, sizeof(ble_slave_tx_buf));

    /* slave receive buffer */
    slave_rx_data_queue.pCurrEntry = ble_slave_rx_buf_0;
    slave_rx_data_queue.pLastEntry = NULL;
    slave_current_rx_entry = ble_slave_rx_buf_0;

    entry = (rfc_dataEntry_t *) ble_slave_rx_buf_0;
    entry->pNextEntry = ble_slave_rx_buf_1;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_slave_rx_buf_0) - 8;

    entry = (rfc_dataEntry_t *) ble_slave_rx_buf_1;
    entry->pNextEntry = ble_slave_rx_buf_0;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_slave_rx_buf_1) - 8;

    /* slave transmit buffer */
    slave_tx_data_queue.pCurrEntry = ble_slave_tx_buf;
    slave_tx_data_queue.pLastEntry = ble_slave_tx_buf;
    slave_current_tx_entry = ble_slave_tx_buf;

    entry = (rfc_dataEntry_t *) ble_slave_tx_buf;
    entry->pNextEntry = NULL;
    entry->config.lenSz = 1;
    entry->length = sizeof(ble_slave_tx_buf) - 8;
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
void create_radio_cmd_now(rfc_bleRadioOp_t *cmd, uint8_t cmd_len, uint16_t command_nr, uint8_t channel,
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
void create_radio_cmd_timestamp(rfc_bleRadioOp_t *cmd, uint8_t cmd_len, uint16_t command_nr, uint8_t channel,
                      uint8_t *params, uint8_t *output, uint32_t start_time)
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
    params->pRxQ = &adv_rx_data_queue;
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
    process_start(&ble_controller_connection_process, NULL);
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
        PROCESS_YIELD_UNTIL(etimer_expired(&adv_timer) || ev == PROCESS_EVENT_POLL);
    }
    PRINTF("ble_controller_advertising_process: advertising stopped\n");
    PROCESS_END();
}

/*---------------------------------------------------------------------------*/
unsigned short ble_controller_read_current_rx_buf(
        void *buffer, unsigned short buffer_length)
{
    int len = 0;
    int8_t rssi;
    uint8_t channel;
    rfc_dataEntryGeneral_t *entry;

    if(state == ADVERTISING)
    {
        PRINTF("ble_controller_read_current_rx_buf() ADV\n");
        entry = (rfc_dataEntryGeneral_t *) adv_current_rx_entry;
    }
    else
    {
        PRINTF("ble_controller_read_current_rx_buf() DATA\n");
        entry = (rfc_dataEntryGeneral_t *) slave_current_rx_entry;
    }

    if(entry->status != DATA_ENTRY_FINISHED)
    {
        return 0;
    }

    len = adv_current_rx_entry[8] - 6;              // last 4 bytes are timestamp
    memcpy(buffer, (char *)&adv_current_rx_entry[9], len);
    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, adv_current_rx_entry[9 + len]);
    packetbuf_set_attr(PACKETBUF_ATTR_CHANNEL, adv_current_rx_entry[9 + len + 1]);

    if(len > 0)
    {
        /* set current entry status to pending */
        entry->status = DATA_ENTRY_PENDING;
        if(state == ADVERTISING) {
            adv_current_rx_entry[8] = 0;
            adv_current_rx_entry = entry->pNextEntry;
        }
        else {
            slave_current_rx_entry[8] = 0;
            slave_current_rx_entry = entry->pNextEntry;
        }
    }


    return len;
}

/*---------------------------------------------------------------------------*/
void ble_controller_free_current_rx_buf()
{
    rfc_dataEntryGeneral_t *entry;
    if(state == ADVERTISING)
    {
        PRINTF("ble_controller_free_current_rx_buf() ADV\n");
        entry = (rfc_dataEntryGeneral_t *)adv_current_rx_entry;
        /* clear the length*/
        adv_current_rx_entry[8] = 0;

        /* set status to pending */
        entry->status = DATA_ENTRY_PENDING;
        adv_current_rx_entry = entry->pNextEntry;
    }
    else
    {
        PRINTF("ble_controller_free_current_rx_buf() DATA\n");
        entry = (rfc_dataEntryGeneral_t *)slave_current_rx_entry;
        /* clear the length*/
        slave_current_rx_entry[8] = 0;

        /* set status to pending */
        entry->status = DATA_ENTRY_PENDING;
        slave_current_rx_entry = entry->pNextEntry;
    }


}

/*---------------------------------------------------------------------------*/
unsigned short parse_connect_request_data()
{
    int data_offset = 23;
    memcpy(&access_address, &adv_current_rx_entry[data_offset], 4);
    crc_init_0 = adv_current_rx_entry[data_offset + 4];
    crc_init_1 = adv_current_rx_entry[data_offset + 5];
    crc_init_2 = adv_current_rx_entry[data_offset + 6];
    win_size = adv_current_rx_entry[data_offset + 7];
    win_offset = (adv_current_rx_entry[data_offset + 9] << 8) + adv_current_rx_entry[data_offset + 8];
    interval = (adv_current_rx_entry[data_offset + 11] << 8) + adv_current_rx_entry[data_offset + 10];
    latency = (adv_current_rx_entry[data_offset + 13] << 8) + adv_current_rx_entry[data_offset + 12];
    timeout = (adv_current_rx_entry[data_offset + 15] << 8) + adv_current_rx_entry[data_offset + 14];
    memcpy(&data_channel_map, &adv_current_rx_entry[data_offset + 16], 5);
    hops = (adv_current_rx_entry[data_offset + 21] & 0x01F);
    sca = (adv_current_rx_entry[data_offset + 21] >> 5) & 0x07;
    conn_req_timestamp_rf_ticks = (adv_current_rx_entry[data_offset + 27] << 24) + (adv_current_rx_entry[data_offset + 26] << 16) +
            (adv_current_rx_entry[data_offset + 25] << 8) + (adv_current_rx_entry[data_offset + 24]);
    current_channel = hops;
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
void create_slave_params(rfc_bleSlavePar_t *params, uint8_t first_packet, uint8_t transmit_data)
{
    uint32_t timeout_time = win_size * 5000;
    if(first_packet) {
        memset(params, 0x00, sizeof(BLE_PARAMS_BUFFER_LENGTH));
    }
    params->pRxQ = &slave_rx_data_queue;

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
//    if(rf_core_wait_cmd_done(cmd) != RF_CORE_CMD_OK) {
//        PRINTF("send_advertisement() could not wait for cmd\n");
//        print_command_status(cmd->status);
//    }
    PRINTF("send_slave_command() ");
    print_command_status(cmd->status);
    return BLE_COMMAND_SUCCESS;
}

/*---------------------------------------------------------------------------*/
void print_data_queue_entry(uint8_t *entry)
{
    int i = 0;
    /* length of the ble data entry */
    /* decrease the length of ble data by rssi, status and timestamp, if included */
    int len = entry[8] - (1 * RX_CONFIG_INCLUDE_RSSI) -
            (1 * RX_CONFIG_INCLUDE_STATUS) - (4 * RX_CONFIG_INCLUDE_TIMESTAMP);
    int ble_offset = 9;
    rfc_dataEntryGeneral_t *e = (rfc_dataEntryGeneral_t *) entry;

    PRINTF("#########################################################\n");
    PRINTF("RX buffer data entry\n");
    PRINTF("status: ");

    switch(e->status)
    {
    case DATA_ENTRY_PENDING:    PRINTF("PENDING "); break;
    case DATA_ENTRY_ACTIVE:     PRINTF("ACTIVE "); break;
    case DATA_ENTRY_BUSY:       PRINTF("BUSY "); break;
    case DATA_ENTRY_FINISHED:   PRINTF("FINISHED "); break;
    case DATA_ENTRY_UNFINISHED: PRINTF("UNFINISHED "); break;
    default:                    PRINTF("unknown "); break;
    }
    PRINTF("\nsize: %d\n", len);
    PRINTF("BLE packet data:");
    for(i = ble_offset; i < ble_offset + len; ++i)
    {
        PRINTF("0x%02X ", entry[i]);
    }

    if(RX_CONFIG_INCLUDE_RSSI)
    {
        PRINTF("\nRSSI: %d", entry[i++]);
    }
    if(RX_CONFIG_INCLUDE_STATUS)
    {
        PRINTF("\nstatus: %d", entry[i++]);
    }
    PRINTF("\n");
}

void conn_event()
{
    rfc_bleRadioOp_t *cmd = (rfc_bleRadioOp_t *) ble_slave_cmd_buf;
    rfc_bleSlavePar_t *params = (rfc_bleSlavePar_t *) ble_slave_params_buf;
    rfc_bleMasterSlaveOutput_t *output = (rfc_bleMasterSlaveOutput_t *) ble_slave_output_buf;

    /* calculate the number of the upcoming connection event */
    conn_event_number++;
    if(conn_event_number >= 2)
    {
        return;
    }
    /* calculate the data channel of the upcoming connection event */
    current_channel = (current_channel + hops) % 37;
    /* calculate the anchor of the upcoming connection event */
    next_anchor_rf_ticks = output->timeStamp + (conn_event_number * interval * 5000);

    create_slave_params(params, 0, 0);
    create_radio_cmd_timestamp(cmd, BLE_COMMAND_BUFFER_LENGTH, CMD_BLE_SLAVE,
                               current_channel, (uint8_t *) params, (uint8_t *) output,
                               next_anchor_rf_ticks);
    send_slave_command(cmd);
    rtimer_set(&conn_timer, RTIMER_NOW() + RTIMER_SECOND/128, 0, conn_event, NULL);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_controller_connection_process, ev, data)
{
    rfc_bleRadioOp_t *cmd = (rfc_bleRadioOp_t *) ble_slave_cmd_buf;
    rfc_bleSlavePar_t *params = (rfc_bleSlavePar_t *) ble_slave_params_buf;
    rfc_bleMasterSlaveOutput_t *output = (rfc_bleMasterSlaveOutput_t *) ble_slave_output_buf;

    PROCESS_BEGIN();
    PRINTF("ble_controller_connection_process started\n");

    /* wait for a CONN REQ */
    PROCESS_YIELD_UNTIL(ev == rf_core_data_event &&
            RF_RADIO_OP_GET_STATUS(ble_adv_cmd_buf) == RF_CORE_RADIO_OP_STATUS_BLE_DONE_CONNECT);
    state = CONNECTED;
    parse_connect_request_data();
    create_slave_params(params, 1, 0);
    create_radio_cmd_timestamp(cmd, BLE_COMMAND_BUFFER_LENGTH, CMD_BLE_SLAVE,
                               hops, (uint8_t *) params, (uint8_t *) output,
                               (conn_req_timestamp_rf_ticks + (win_offset * 5000)));

    /* send slave command to rf-core */
    send_slave_command(cmd);
    /* wait for first connection event to be finished */
    PROCESS_WAIT_UNTIL(ev == rf_core_data_event &&
            cmd->status == RF_CORE_RADIO_OP_STATUS_BLE_DONE_OK);

    /* after successfully exchanging packets, the following conditions are met: */
    /* - the output.timestamp is valid and stores the anchor for all following connection events */
    if(output->pktStatus.bTimeStampValid != 1)
    {
        PRINTF("ble_controller_connection_process no timestamp available\n");
        PROCESS_EXIT();
    }

    while(1)
    {
//        rtimer_set(&conn_timer, RTIMER_NOW() + RTIMER_SECOND/32, 0, conn_event, NULL);
        PROCESS_YIELD();
    }
    PROCESS_END();
}
