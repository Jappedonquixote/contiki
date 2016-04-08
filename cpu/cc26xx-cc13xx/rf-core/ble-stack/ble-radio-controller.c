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
 * ble-radio-controller.c
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "contiki.h"

#include "lpm.h"

#include "ble-addr.h"
#include "ble-radio-controller.h"
#include "ble-radio-advertising.h"

#include "dev/radio.h"
#include "dev/oscillators.h"

#include "net/netstack.h"
#include "net/packetbuf.h"

#include "rf-core/api/data_entry.h"
#include "rf-core/api/ble_cmd.h"
#include "rf-core/api/common_cmd.h"
#include "rf-core/ble-stack/ble-addr.h"
#include "rf-core/rf-core.h"
#include "rf-core/rf-core-debug.h"

#include "lib/memb.h"
#include "lib/list.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Defines the ticks, which the system CPU needs to wakeup before an
 * upcoming connection event anchor point */
#define WAKEUP_BEFORE_ANCHOR_TICKS 12000     /* 3 ms */

/* The radio core starts the slave operation START_BEFORE_ANCHOR_TICKS
 * before the real anchor point */
#define START_BEFORE_ANCHOR_TICKS  4000     /* 1 ms */

#define STOP_AT_X_EVENTS           1000

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
/* BLE connection request data (timing values stored in rf-core ticks) */
static uint32_t access_address;
static uint8_t crc_init_0;
static uint8_t crc_init_1;
static uint8_t crc_init_2;
static uint32_t win_size_ticks;
static uint32_t win_offset_ticks;
static uint32_t interval_ticks;
static uint8_t hop;

static uint32_t conn_req_timestamp;

static uint8_t current_channel;
static uint32_t current_event_number;

static uint8_t first_anchor_point_valid;
static uint32_t first_anchor_point_ticks;
static uint32_t current_anchor_point_ticks;
static uint32_t next_anchor_point_ticks;
/*---------------------------------------------------------------------------*/
/* rf-core slave command, params and output */
static rfc_CMD_BLE_SLAVE_t slave_cmd;
static rfc_bleSlavePar_t slave_param;
static rfc_bleMasterSlaveOutput_t slave_output;
/*---------------------------------------------------------------------------*/
/* slave command receive buffer */
static uint8_t rx_buf_0[BLE_RADIO_RX_BUF_LEN] CC_ALIGN(4);
static uint8_t rx_buf_1[BLE_RADIO_RX_BUF_LEN] CC_ALIGN(4);
static uint8_t rx_buf_2[BLE_RADIO_RX_BUF_LEN] CC_ALIGN(4);
static uint8_t rx_buf_3[BLE_RADIO_RX_BUF_LEN] CC_ALIGN(4);

static dataQueue_t rx_data_queue = { 0 };
static uint8_t *current_rx_entry;
/*---------------------------------------------------------------------------*/
/* slave command transmit buffer */
typedef struct {
    rfc_dataEntry_t entry;
    uint8_t data[BLE_RADIO_TX_BUF_LEN];
} tx_buffer_t;

static dataQueue_t tx_data_queue = { 0 };

/* a list of all currently used tx buffers                                   */
LIST(tx_buffers_list);
/* memory block for allocation of tx buffers                                 */
MEMB(tx_buffers,tx_buffer_t, 4);
/*---------------------------------------------------------------------------*/
/* TODO: remove the variables for timemeasuring*/
//#define TIMESTAMP_LOCATION 0x40043004
//#define READ_CURRENT_TIME(x) (memcpy(x, TIMESTAMP_LOCATION, 4))
//#define TICKS_TO_MS(x) ((x) / 4000)
//static uint32_t start_time;
//static uint32_t end_time;
/*---------------------------------------------------------------------------*/
PROCESS(ble_radio_controller, "BLE/CC26xx process");
/*---------------------------------------------------------------------------*/
static uint8_t
request(void)
{
    if(rf_core_is_accessible()) {
        return LPM_MODE_SLEEP;
    }

    return LPM_MODE_MAX_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
LPM_MODULE(cc26xx_ble_lpm_module, request, NULL, NULL, LPM_DOMAIN_NONE);
/*---------------------------------------------------------------------------*/
unsigned short setup_ble_mode(void)
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
        return BLE_RADIO_CMD_ERROR;
    }

    /* Wait until radio setup is done */
    if(rf_core_wait_cmd_done(&cmd) != RF_CORE_CMD_OK) {
        PRINTF("setup_ble_mode() wait: ");
        print_command_status(cmd.status);
        return BLE_RADIO_CMD_ERROR;
    }

    return BLE_RADIO_CMD_OK;
}

/*---------------------------------------------------------------------------*/
void setup_slave_buffers(void)
{
    memb_init(&tx_buffers);
    list_init(tx_buffers_list);

    memset(&slave_cmd, 0x00, sizeof(slave_cmd));
    memset(&slave_param, 0x00, sizeof(slave_param));
    memset(&slave_output, 0x00, sizeof(slave_output));

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
unsigned short ble_radio_controller_init(void)
{
    PRINTF("ble_radio_controller_init()\n");

    /* read public BLE address from device */
    ble_addr_init();

    /* register low power module */
    lpm_register_module(&cc26xx_ble_lpm_module);

    /* set the mode in the rf core */
    rf_core_set_modesel();

    setup_slave_buffers();

    oscillators_request_hf_xosc();

    if(rf_core_is_accessible())
    {
        /* rf core is already accessible */
        process_start(&ble_radio_controller, NULL);
        return BLE_RADIO_CMD_OK;
    }

    /* boot the rf core */
    if(rf_core_boot() != RF_CORE_CMD_OK)
    {
        PRINTF("ble_radio_controller_init() could not boor rf-core\n");
        return BLE_RADIO_CMD_ERROR;
    }

    rf_core_setup_interrupts();
    oscillators_switch_to_hf_xosc();

    if(setup_ble_mode() != BLE_RADIO_CMD_OK)
    {
        PRINTF("ble_radio_controller_init() could not setup rf-core to BLE\n");
        return BLE_RADIO_CMD_ERROR;
    }

    process_start(&ble_radio_controller, NULL);

    return BLE_RADIO_CMD_OK;
}

/*---------------------------------------------------------------------------*/
void parse_conn_req_data(ble_conn_req_data_t *data)
{
    access_address = data->access_address;
    crc_init_0 = data->crc_init_0;
    crc_init_1 = data->crc_init_1;
    crc_init_2 = data->crc_init_2;
    win_size_ticks = (uint32_t) (data->win_size * 5000);
    win_offset_ticks = (uint32_t) (data->win_offset * 5000);
    interval_ticks = (uint32_t) (data->interval * 5000);
    hop = data->hop;
    conn_req_timestamp = data->timestamp;
}

/*---------------------------------------------------------------------------*/
void create_slave_cmd(rfc_CMD_BLE_SLAVE_t *cmd, uint8_t channel,
                      rfc_bleSlavePar_t *params, rfc_bleMasterSlaveOutput_t *output,
                      uint32_t start_time)
{
    memset(cmd, 0x00, sizeof(rfc_CMD_BLE_SLAVE_t));
    cmd->commandNo = CMD_BLE_SLAVE;
    cmd->condition.rule = COND_NEVER;
    cmd->whitening.bOverride = 0;
    cmd->channel = channel;
    cmd->pParams = params;
    cmd->startTrigger.triggerType = TRIG_ABSTIME;
    cmd->startTrigger.pastTrig = 0;
    cmd->startTime = start_time;
    cmd->pOutput = output;
}

/*---------------------------------------------------------------------------*/
void create_slave_params(rfc_bleSlavePar_t *params, uint8_t first_packet)
{
//    uint32_t timeout_ticks = (interval_ticks / 2);
    uint32_t timeout_ticks = win_size_ticks + START_BEFORE_ANCHOR_TICKS;

    params->pRxQ = &rx_data_queue;
    params->pTxQ = &tx_data_queue;

    params->rxConfig.bAutoFlushIgnored = 0;
    params->rxConfig.bAutoFlushCrcErr = 0;
    params->rxConfig.bAutoFlushEmpty = 0;
    params->rxConfig.bIncludeLenByte = 1;
    params->rxConfig.bIncludeCrc = 0;
    params->rxConfig.bAppendRssi = 1;
    params->rxConfig.bAppendStatus = 1;
    params->rxConfig.bAppendTimestamp = 1;

    if(first_packet) {
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
    params->timeoutTime = timeout_ticks;
    params->endTrigger.triggerType = TRIG_NEVER;
}

/*---------------------------------------------------------------------------*/
uint8_t send_slave_command(rfc_CMD_BLE_SLAVE_t * cmd)
{
    uint32_t cmd_status;
    if(rf_core_send_cmd((uint32_t)cmd, &cmd_status) != RF_CORE_CMD_OK) {
        PRINTF("send_slave_command() could not send cmd to rf-core: \n");
        print_cmdsta(cmd_status);
        print_command_status(cmd->status);
        return BLE_RADIO_CMD_ERROR;
    }
    return BLE_RADIO_CMD_OK;
}

/*---------------------------------------------------------------------------*/
void process_current_rx_data_buf(void)
{
    uint8_t data_offset = 9;                     // start index of BLE data
    uint8_t data_len;
    uint8_t rssi;
    uint8_t channel;

    rfc_dataEntryGeneral_t *entry = (rfc_dataEntryGeneral_t *) current_rx_entry;
    if(entry->status != DATA_ENTRY_FINISHED)
    {
        return;
    }

    /* clear the packetbuffer */
    packetbuf_clear();

    /* the rx buffer includes the ble data and appends 1 status byte,
     * 1 rssi byte and 4 timestamp bytes
     * we do not need the 6 appended bytes in the payload */
    data_len = current_rx_entry[8] - 6;

    if(data_len > 0)
    {
        /* copy payload in packetbuffer */
        memcpy(packetbuf_dataptr(), &current_rx_entry[9], data_len);
        rssi = current_rx_entry[data_offset + data_len];
        channel = (current_rx_entry[data_offset + data_len + 1] & 0x1F);
        packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rssi);
        packetbuf_set_attr(PACKETBUF_ATTR_CHANNEL, channel);

        packetbuf_set_datalen(data_len);
        NETSTACK_RDC.input();
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
void ble_radio_controller_send(const uint8_t *payload, uint8_t payload_len)
{
    tx_buffer_t *buf;
    uint32_t cmdsta;

    /* allocate a tx_buf for the packet */
    buf = memb_alloc(&tx_buffers);
    if(buf == NULL)
    {
        PRINTF("ble_radio_controller_send: could not allocate buffer\n");
        return;
    }
    list_add(tx_buffers_list, buf);

    /* write payload to buf */
    buf->entry.length = payload_len;
    buf->entry.config.lenSz = 1;
    buf->entry.pNextEntry = NULL;
    memcpy(buf->data, payload, payload_len);

    rfc_CMD_ADD_DATA_ENTRY_t cmd_add_buf;
    cmd_add_buf.commandNo = CMD_ADD_DATA_ENTRY;
    cmd_add_buf.pQueue = &tx_data_queue;
    cmd_add_buf.pEntry = (uint8_t *) buf;

    if(rf_core_send_cmd((uint32_t) &cmd_add_buf, &cmdsta) != RF_CORE_CMD_OK)
    {
        print_cmdsta(cmdsta);
    }
}

/*---------------------------------------------------------------------------*/
void free_finished_tx_buffers(void)
{
    tx_buffer_t *buf = list_head(tx_buffers_list);

    /* free all finished tx buffers */
    while((buf != NULL) && (buf->entry.status == DATA_ENTRY_FINISHED))
    {
        /* free the buffer memory */
        memb_free(&tx_buffers, buf);

        /* remove buffer from buffer list */
        list_pop(tx_buffers_list);

        buf = list_head(tx_buffers_list);
    }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_radio_controller, ev, data)
{
    ble_conn_req_data_t *conn_req_data;
    uint8_t result;

    PROCESS_BEGIN();
    PRINTF("ble_radio_controller_process() started\n");

    ble_radio_advertising_enable_advertising();
    while(1)
    {
        PROCESS_WAIT_EVENT();
        /* CONN REQ data received */
        if(ev == conn_req_event)
        {
            /* parse conn req data */
            conn_req_data = (ble_conn_req_data_t *) data;
            parse_conn_req_data(conn_req_data);

            current_event_number = 0;
            current_channel = hop;

            if(win_offset_ticks <= 60000)
            {
                /* in this case the first anchor point starts too early,
                 * ignore the first conn event and start with the 2nd */
                first_anchor_point_ticks = conn_req_timestamp + win_offset_ticks + interval_ticks;
                current_channel = (current_channel + hop) % 37;
            }
            else
            {
                first_anchor_point_ticks = conn_req_timestamp + win_offset_ticks;
            }

            /* the first anchor point is currently only an approximation */
            /* the exact value will be read from the first exchanged connection event */
            first_anchor_point_valid = 0;

            create_slave_params(&slave_param, 1);
            create_slave_cmd(&slave_cmd, current_channel,
                             &slave_param, &slave_output,
                             (first_anchor_point_ticks - START_BEFORE_ANCHOR_TICKS));
            result = send_slave_command(&slave_cmd);
            if(result != BLE_RADIO_CMD_OK)
            {
                PRINTF("ble-radio-controller: could not establish connection\n");
                PROCESS_EXIT();
            }
            next_anchor_point_ticks = first_anchor_point_ticks + interval_ticks;
            rf_core_start_timer_comp(next_anchor_point_ticks
                                     - WAKEUP_BEFORE_ANCHOR_TICKS);
        }
        /* CONNECTION EVENT */
        if(ev == rf_core_timer_event)
        {
            /* calculate parameters for upcoming connection event */
            current_anchor_point_ticks = next_anchor_point_ticks;
            current_event_number++;
            current_channel = (current_channel + hop) % 37;

            /* create & send slave command for upcoming connection event */
            create_slave_params(&slave_param, 0);
            create_slave_cmd(&slave_cmd, current_channel,
                             &slave_param, &slave_output,
                             current_anchor_point_ticks - START_BEFORE_ANCHOR_TICKS);
            result = send_slave_command(&slave_cmd);
            if(result != BLE_RADIO_CMD_OK)
            {
                PRINTF("ble-radio-controller: connection error; event-nr.: %lu\n",
                       current_event_number);
                PROCESS_EXIT();
            }

            /* calculate next anchor point & setup timer interrupt */
            next_anchor_point_ticks = first_anchor_point_ticks +
                                      (interval_ticks * (current_event_number + 1));
            rf_core_start_timer_comp(next_anchor_point_ticks
                                     - WAKEUP_BEFORE_ANCHOR_TICKS);

            free_finished_tx_buffers();
        }
        /* RX data entry available */
        if(ev == rf_core_data_event)
        {
            if((!first_anchor_point_valid) && (slave_output.pktStatus.bTimeStampValid))
            {
                /* the first anchor point is now the valid value from the 1st conn event */
                first_anchor_point_valid = 1;
                first_anchor_point_ticks = slave_output.timeStamp;
            }
            process_current_rx_data_buf();
        }
    }

    PRINTF("ble_radio_controller_process() stopped\n");
    PROCESS_END();
}
