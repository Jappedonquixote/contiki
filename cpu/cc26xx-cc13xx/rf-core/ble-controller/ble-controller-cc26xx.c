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

#include "lpm.h"

#include "sys/process.h"

#include "dev/ble-controller.h"
#include "dev/oscillators.h"

#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/framer-ble.h"
#include "net/frame-ble.h"

#include "rf-core/api/data_entry.h"
#include "rf-core/rf-core.h"
#include "rf-core/ble-controller/rf-ble-cmd.h"

#include <string.h>

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/**
 * The location of the primary BLE device address (public BLE address)
 */
#define BLE_ADDR_LOCATION   0x500012E8
/*---------------------------------------------------------------------------*/
/* The size of a single BLE data buffer*/
#define BLE_CONTROLLER_DATA_BUF_SIZE          80
/* The number of single BLE data buffers*/
#define BLE_CONTROLLER_NUM_DATA_BUF            4
/*---------------------------------------------------------------------------*/
typedef uint32_t rf_ticks_t;
/*---------------------------------------------------------------------------*/
/* ADVERTISING                                                               */
typedef struct {
    rf_ticks_t interval;
    ble_adv_type_t type;
    ble_addr_type_t own_addr_type;
    ble_addr_type_t dir_addr_type;
    ble_addr_t dir_addr;
    uint8_t channel_map;
} ble_adv_param_t;

/* advertising parameter */
static ble_adv_param_t adv_param;

/* advertising data */
static uint8_t adv_data_len;
static uint8_t adv_data[BLE_ADV_DATA_LEN];

/* scan response data */
static uint8_t scan_resp_data_len;
static uint8_t scan_resp_data[BLE_SCAN_RESP_DATA_LEN];

/* timing values */
static rf_ticks_t adv_event_next;
/*---------------------------------------------------------------------------*/
/* CONNECTION                                                                */
#define CONN_EVENT_START_BEFORE_ANCHOR   4000   /* 1 ms */
#define CONN_EVENT_WAKEUP_BEFORE_ANCHOR 12000   /* 3 ms*/

typedef struct {
    uint32_t access_address;
    uint8_t crc_init_0;
    uint8_t crc_init_1;
    uint8_t crc_init_2;
    rf_ticks_t win_size;
    rf_ticks_t win_offset;
    rf_ticks_t interval;
    uint16_t latency;
    rf_ticks_t timeout;
    uint64_t channel_map;
    uint8_t hop;
    uint8_t sca;
    rf_ticks_t timestamp;
} ble_conn_param_t;

/* connection parameter */
static ble_conn_param_t conn_param;

typedef struct {
    /* connection event counter */
    uint16_t counter;

    /* used data channel */
    uint8_t channel;

    /* start of the connection event */
    rf_ticks_t start;

    /* start of the next connection event */
    rf_ticks_t next_start;
} ble_conn_event_t;

/* connection event data */
static ble_conn_event_t conn_event;
static rf_ticks_t first_conn_event_anchor;
/*---------------------------------------------------------------------------*/
/* RX data queue (all received packets are stored in the same queue)         */
#define BLE_RX_BUF_OVERHEAD 8
#define BLE_RX_BUF_LEN      (BLE_CONTROLLER_DATA_BUF_SIZE + BLE_RX_BUF_OVERHEAD)
#define BLE_RX_NUM_BUF      BLE_CONTROLLER_NUM_DATA_BUF

static uint8_t rx_bufs[BLE_RX_NUM_BUF][BLE_RX_BUF_LEN] CC_ALIGN(4);

static dataQueue_t rx_data_queue = { 0 };
volatile static uint8_t *current_rx_entry;
/*---------------------------------------------------------------------------*/
/* TX data queue (data channel packets are stored in the same queue)         */
#define BLE_TX_BUF_OVERHEAD 8
#define BLE_TX_BUF_LEN      (BLE_CONTROLLER_DATA_BUF_SIZE + BLE_TX_BUF_OVERHEAD)
#define BLE_TX_NUM_BUF      BLE_CONTROLLER_NUM_DATA_BUF

static uint8_t tx_bufs[BLE_TX_NUM_BUF][BLE_TX_BUF_LEN] CC_ALIGN(4);

static dataQueue_t tx_data_queue = { 0 };
volatile static uint8_t *current_tx_entry;
/*---------------------------------------------------------------------------*/
typedef enum {
    BLE_CONTROLLER_STATE_STANDBY,
    BLE_CONTROLLER_STATE_ADVERTISING,
    BLE_CONTROLLER_STATE_SCANNING,
    BLE_CONTROLLER_STATE_INITIATING,
    BLE_CONTROLLER_STATE_CONN_MASTER,
    BLE_CONTROLLER_STATE_CONN_SLAVE
} ble_controller_state_t;

/* The link layer state of the ble controller */
static ble_controller_state_t state = BLE_CONTROLLER_STATE_STANDBY;
/*---------------------------------------------------------------------------*/
/* RF core buffers                                                           */
#define RF_CMD_BUFFER_SIZE    128
#define RF_PARAM_BUFFER_SIZE   80
/* buffer for all commands to the RF core */
static uint8_t cmd_buf[RF_CMD_BUFFER_SIZE];
/* buffer for all command parameters to the RF core */
static uint8_t param_buf[RF_PARAM_BUFFER_SIZE];
/*---------------------------------------------------------------------------*/
/* LPM                                                                       */
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

/*---------------------------------------------------------------------------*/
PROCESS(ble_controller_process, "BLE/CC26xx process");
/*---------------------------------------------------------------------------*/
static void setup_buffers(void)
{
    rfc_dataEntry_t *entry;
    uint8_t i;

    /* setup circular RX buffer */
    rx_data_queue.pCurrEntry = rx_bufs[0];
    rx_data_queue.pLastEntry = NULL;
    current_rx_entry = rx_bufs[0];

    /* initialize each individual rx buffer entry */
    for(i = 0; i < BLE_RX_NUM_BUF; i++) {
        memset(rx_bufs[i], 0x00, BLE_RX_BUF_LEN);
        entry = (rfc_dataEntry_t *) rx_bufs[i];
        entry->pNextEntry = rx_bufs[(i + 1) % BLE_RX_NUM_BUF];
        entry->config.lenSz = 1;
        entry->length = BLE_RX_BUF_LEN - BLE_RX_BUF_OVERHEAD;
    }

    /* setup circular TX buffer */
    tx_data_queue.pCurrEntry = tx_bufs[0];
    tx_data_queue.pLastEntry = NULL;
    current_tx_entry = tx_bufs[0];

    /* initialize each individual tx buffer entry */
    for(i = 0; i < BLE_TX_NUM_BUF; i++) {
        memset(tx_bufs[i], 0x00, BLE_TX_BUF_LEN);
        entry = (rfc_dataEntry_t *) tx_bufs[i];
        entry->pNextEntry = tx_bufs[(i + 1) % BLE_TX_NUM_BUF];
        entry->config.lenSz = 1;
        entry->length = 0;
    }
}
/*---------------------------------------------------------------------------*/
static ble_result_t reset(void)
{
    /* register low power module */
    lpm_register_module(&cc26xx_ble_lpm_module);

    /* set the mode in the rf core */
    rf_core_set_modesel();

    setup_buffers();

    oscillators_request_hf_xosc();

    if(!rf_core_is_accessible())
    {
        /* boot the rf core */
        if(rf_core_boot() != RF_CORE_CMD_OK)
        {
            PRINTF("ble_controller_reset() could not boot rf-core\n");
            return BLE_RESULT_ERROR;
        }

        rf_core_setup_interrupts();
        oscillators_switch_to_hf_xosc();

        if(rf_ble_cmd_setup_ble_mode() != RF_BLE_CMD_OK) {
            PRINTF("could not setup rf-core to BLE mode\n");
            return BLE_RESULT_ERROR;
        }
    }

    state = BLE_CONTROLLER_STATE_STANDBY;

    /* start the BLE controller process, if it is not currently running */
    if(!process_is_running(&ble_controller_process)) {
        process_start(&ble_controller_process, NULL);
    }
    return BLE_RESULT_OK;
}

/*---------------------------------------------------------------------------*/
static ble_result_t read_bd_addr(ble_addr_t *addr)
{
    memcpy(addr, (uint8_t *) BLE_ADDR_LOCATION, BLE_ADDR_SIZE);
    return BLE_RESULT_OK;
}

/*---------------------------------------------------------------------------*/
static ble_result_t read_buffer_size(unsigned int *buf_len,
        unsigned int *num_buf)
{
    *buf_len = BLE_CONTROLLER_DATA_BUF_SIZE;
    *num_buf = BLE_CONTROLLER_NUM_DATA_BUF;
    return BLE_RESULT_OK;
}

/*---------------------------------------------------------------------------*/
static ble_result_t set_adv_param(unsigned int adv_int, ble_adv_type_t type,
        ble_addr_type_t own_type, ble_addr_type_t dir_type, ble_addr_t dir,
        unsigned short adv_map)
{
    /* convert the adv_int according to BLE standard to rf core ticks */
    adv_param.interval = adv_int * 2500;
    adv_param.type = type;
    adv_param.own_addr_type = own_type;
    adv_param.dir_addr_type = dir_type;
    memcpy(&adv_param.dir_addr, &dir, BLE_ADDR_SIZE);
    adv_param.channel_map = adv_map;

    return BLE_RESULT_OK;
}

/*---------------------------------------------------------------------------*/
static ble_result_t read_adv_channel_tx_power(short *power)
{
    // TODO
    return BLE_RESULT_NOT_SUPPORTED;
}

/*---------------------------------------------------------------------------*/
static ble_result_t set_adv_data(unsigned short data_len, char *data)
{
    if(data_len > BLE_ADV_DATA_LEN) {
        return BLE_RESULT_INVALID_PARAM;
    }
    adv_data_len = data_len;
    memcpy(adv_data, data, data_len);
    return BLE_RESULT_OK;
}

/*---------------------------------------------------------------------------*/
static ble_result_t set_scan_resp_data(unsigned short data_len, char *data)
{
    if(data_len > BLE_SCAN_RESP_DATA_LEN) {
        return BLE_RESULT_INVALID_PARAM;
    }
    scan_resp_data_len = data_len;
    memcpy(scan_resp_data, data, data_len);
    return BLE_RESULT_OK;
}

/*---------------------------------------------------------------------------*/
static ble_result_t set_adv_enable(unsigned short enable)
{
    if((enable == 1) && (state == BLE_CONTROLLER_STATE_STANDBY)) {
        state = BLE_CONTROLLER_STATE_ADVERTISING;

        /* start the timer for advertising events */
        adv_event_next = rf_core_read_current_rf_ticks() + adv_param.interval;
        rf_core_start_timer_comp(adv_event_next);

        return BLE_RESULT_OK;
    }
    else if((enable != 1) && (state == BLE_CONTROLLER_STATE_ADVERTISING)) {
        state = BLE_CONTROLLER_STATE_STANDBY;

        return BLE_RESULT_OK;
    }
    return BLE_RESULT_ERROR;
}

/*---------------------------------------------------------------------------*/
static ble_result_t set_scan_param(ble_scan_type_t type,
                                    unsigned int scan_interval,
                                    unsigned int scan_window,
                                    ble_addr_type_t own_addr_type)
{
    // TODO
    return BLE_RESULT_NOT_SUPPORTED;
}

/*---------------------------------------------------------------------------*/
static ble_result_t set_scan_enable(unsigned short enable,
                                     unsigned short filter_duplicates)
{
    // TODO
    return BLE_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static ble_result_t create_connection(unsigned int scan_interval,
                                         unsigned int scan_window,
                                         ble_addr_type_t peer_addr_type,
                                         ble_addr_t peer_addr,
                                         ble_addr_type_t own_addr_type,
                                         unsigned int conn_interval,
                                         unsigned int conn_latency,
                                         unsigned int supervision_timeout)
{
    // TODO
    return BLE_RESULT_NOT_SUPPORTED;
}

/*---------------------------------------------------------------------------*/
static ble_result_t create_connection_cancel(void)
{
    // TODO
    return BLE_RESULT_NOT_SUPPORTED;
}

/*---------------------------------------------------------------------------*/
static ble_result_t connection_update(unsigned int connection_handle,
                                         unsigned int conn_interval,
                                         unsigned int conn_latency,
                                         unsigned int supervision_timeout)
{
    // TODO
    return BLE_RESULT_NOT_SUPPORTED;
}

/*---------------------------------------------------------------------------*/
static ble_result_t disconnect(unsigned int connection_handle,
                               unsigned short reason)
{
    // TODO
    return BLE_RESULT_NOT_SUPPORTED;
}

/*---------------------------------------------------------------------------*/
static ble_result_t queue_tx_data(unsigned short data_len, char *data)
{
    // TODO
    return BLE_RESULT_NOT_SUPPORTED;
}

/*---------------------------------------------------------------------------*/
const struct ble_controller_driver ble_controller =
{
    reset,
    read_bd_addr,
    read_buffer_size,
    set_adv_param,
    read_adv_channel_tx_power,
    set_adv_data,
    set_scan_resp_data,
    set_adv_enable,
    set_scan_param,
    set_scan_enable,
    create_connection,
    create_connection_cancel,
    connection_update,
    disconnect,
    queue_tx_data
};


/*---------------------------------------------------------------------------*/
/* The parameter are parsed according to Bluetooth Specification v4 (page 2510)*/
static void parse_connect_request_data(ble_conn_param_t *p, uint8_t *entry)
{
    int offset = 0;
    memcpy(&p->access_address, &entry[offset], 4);
    p->crc_init_0 = entry[offset + 4];
    p->crc_init_1 = entry[offset + 5];
    p->crc_init_2 = entry[offset + 6];
    p->win_size = entry[offset + 7];
    p->win_offset = (entry[offset + 9] << 8) + entry[offset + 8];
    p->interval = (entry[offset + 11] << 8) + entry[offset + 10];
    p->latency = (entry[offset + 13] << 8) + entry[offset + 12];
    p->timeout = (entry[offset + 15] << 8) + entry[offset + 14];
    memcpy(&p->channel_map, &entry[offset + 16], 5);
    p->hop = entry[offset + 21] & 0x1F;
    p->sca = (entry[offset + 21] >> 5) & 0x07;
    p->timestamp = (entry[offset + 27] << 24) +
                              (entry[offset + 26] << 16) +
                              (entry[offset + 25] << 8) +
                               entry[offset + 24];
}
/*---------------------------------------------------------------------------*/
static void free_finished_rx_bufs(void)
{
    rfc_dataEntryGeneral_t *entry;

    /* free finished RX entries */
    entry = (rfc_dataEntryGeneral_t *) current_rx_entry;
    while(entry->status == DATA_ENTRY_FINISHED) {
        /* clear the length field */
        current_rx_entry[8] = 0;
        /* set status to pending */
        entry->status = DATA_ENTRY_PENDING;
        /* set next data queue entry */
        current_rx_entry = entry->pNextEntry;
        entry = (rfc_dataEntryGeneral_t *) current_rx_entry;
    }
}
/*---------------------------------------------------------------------------*/
static void free_finished_tx_bufs(void)
{
    rfc_dataEntryGeneral_t *entry;

    /* free finished TX entries */
    entry = (rfc_dataEntryGeneral_t *) current_tx_entry;
    while(entry->status == DATA_ENTRY_FINISHED) {
        PRINTF("finished TX entry\n");
        /* clear the length field */
        current_tx_entry[8] = 0;
        /* set status to pending */
        entry->status = DATA_ENTRY_PENDING;
        /* set next data queue entry */
        current_tx_entry = entry->pNextEntry;
        entry = (rfc_dataEntryGeneral_t *) current_tx_entry;
    }
}
/*---------------------------------------------------------------------------*/
static void state_advertising(process_event_t ev, process_data_t data,
                              uint8_t *cmd, uint8_t *param)
{
    rfc_dataEntryGeneral_t *entry;

    if(ev == rf_core_timer_event) {
        /* advertising event */
        rf_ble_cmd_create_adv_params(param, &rx_data_queue, adv_data_len,
                adv_data, scan_resp_data_len, scan_resp_data,
                adv_param.own_addr_type, (char *) BLE_ADDR_LOCATION);

        if (adv_param.channel_map & BLE_ADV_CHANNEL_1_MASK) {
            rf_ble_cmd_create_adv_cmd(cmd, BLE_ADV_CHANNEL_1, param, NULL);
            rf_ble_cmd_send(cmd);
            rf_ble_cmd_wait(cmd);
        }
        if (adv_param.channel_map & BLE_ADV_CHANNEL_2_MASK) {
            rf_ble_cmd_create_adv_cmd(cmd, BLE_ADV_CHANNEL_2, param, NULL);
            rf_ble_cmd_send(cmd);
            rf_ble_cmd_wait(cmd);
        }
        if (adv_param.channel_map & BLE_ADV_CHANNEL_3_MASK) {
            rf_ble_cmd_create_adv_cmd(cmd, BLE_ADV_CHANNEL_3, param, NULL);
            rf_ble_cmd_send(cmd);
            rf_ble_cmd_wait(cmd);
        }

        adv_event_next = adv_event_next + adv_param.interval;
        rf_core_start_timer_comp(adv_event_next);
    } else if(ev == rf_core_data_rx_event) {
        /* data received */
        entry = (rfc_dataEntryGeneral_t *) current_rx_entry;
        if(entry->status != DATA_ENTRY_FINISHED)
        {
            return;
        }

        if((current_rx_entry[9] & 0x0F) == 5) {
            /* CONN REQ received */

            /* switch to the standby state, until connection data is parsed */
            state = BLE_CONTROLLER_STATE_STANDBY;

            /* parse connection data*/
            parse_connect_request_data(&conn_param,
                    (uint8_t *) &current_rx_entry[23]);

            /* convert the timing values into rf core ticks */
            conn_param.win_size = conn_param.win_size * 5000;
            conn_param.win_offset = conn_param.win_offset * 5000;
            conn_param.interval = conn_param.interval * 5000;
            conn_param.timeout = conn_param.timeout * 40000;

            conn_event.counter = 0;
            conn_event.channel = conn_param.hop;
            first_conn_event_anchor = conn_param.timestamp + conn_param.win_offset;

            if(conn_param.win_offset <= 60000) {
                /* in this case the first anchor point starts too early,
                 * ignore the first conn event and start with the 2nd */
                conn_event.counter++;
                conn_event.channel = (conn_event.channel + conn_param.hop) % 37;
                first_conn_event_anchor += conn_param.interval;
            }
            conn_event.start = first_conn_event_anchor;

            rf_ble_cmd_create_slave_params(param, &rx_data_queue, NULL,
                    conn_param.access_address, conn_param.crc_init_0,
                    conn_param.crc_init_1, conn_param.crc_init_2,
                    conn_param.win_size, CONN_EVENT_START_BEFORE_ANCHOR, 1);

            rf_ble_cmd_create_slave_cmd(cmd, conn_event.channel, param,
                    NULL, (conn_event.start - CONN_EVENT_START_BEFORE_ANCHOR));

            if (rf_ble_cmd_send(cmd) != RF_BLE_CMD_OK)
            {
                PRINTF("could not establish connection\n");
                return;
            }

            conn_event.next_start = first_conn_event_anchor + conn_param.interval;
            rf_core_start_timer_comp(conn_event.next_start - CONN_EVENT_WAKEUP_BEFORE_ANCHOR);

            state = BLE_CONTROLLER_STATE_CONN_SLAVE;
        }
        free_finished_rx_bufs();
    }
}
/*---------------------------------------------------------------------------*/
static void process_llid_control_mesg(uint8_t *payload)
{
    uint8_t opcode = payload[0];
    uint8_t response_data[26];
    uint8_t response_len = 0;

    if(opcode == FRAME_BLE_LL_FEATURE_REQ)
    {
        PRINTF("LL_FEATURE_REQ received\n");
        /* set LLID control frame */
        response_data[0] = 0x03;

        /* set LLID opcode*/
        response_data[1] = FRAME_BLE_LL_FEATURE_RSP;

        /* set feature_set to 0 (no feature supported) */
        memset(&response_data[2], 0x00, 8);

        response_len = 10;
//        ble_radio_controller_send(response_data, response_len);
    }
    else if(opcode == FRAME_BLE_LL_VERSION_IND)
    {
        PRINTF("LL_VERSION_IND received\n");
        /* set LLID control frame */
        response_data[0] = 0x03;

        /* set LLID opcode*/
        response_data[1] = FRAME_BLE_LL_VERSION_IND;

        // TODO: send the real version and company ID
        response_data[2] = 0x06;
        response_data[3] = 0x00;
        response_data[4] = 0x0A;
        response_data[5] = 0xFE;
        response_data[6] = 0xCA;

        response_len = 7;
//        ble_radio_controller_send(response_data, response_len);
    }
    else
    {
        PRINTF("control frame (opcode: 0x%0X) received\n",
                opcode);
    }
}
/*---------------------------------------------------------------------------*/
static void process_rx_entry_data_channel(void)
{
    uint8_t data_offset = 9;                     // start index of BLE data
    uint8_t data_len;
    uint8_t hdr_len;
    uint8_t rssi;
    uint8_t channel;

    rfc_dataEntryGeneral_t *entry = (rfc_dataEntryGeneral_t *) current_rx_entry;
    if(entry->status != DATA_ENTRY_FINISHED)
    {
        return;
    }

    /* clear the packetbuffer */
    packetbuf_clear();

    /* the last 6 bytes of the data are status and timestamp bytes */
    data_len = current_rx_entry[8] - 6;

    if(data_len > 0)
    {
        /* copy payload in packetbuffer */
        memcpy(packetbuf_dataptr(), (uint8_t *) &current_rx_entry[9], data_len);

        /* set the controller dependent attributes */
        rssi = current_rx_entry[data_offset + data_len];
        channel = (current_rx_entry[data_offset + data_len + 1] & 0x1F);
        packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rssi);
        packetbuf_set_attr(PACKETBUF_ATTR_CHANNEL, channel);

        packetbuf_set_datalen(data_len);
        hdr_len = NETSTACK_CONF_FRAMER.parse();

        if(hdr_len < 0)
        {
            PRINTF("could not parse data channel packet\n");
            return;
        }

        if(packetbuf_attr(PACKETBUF_ATTR_FRAME_TYPE) == FRAME_BLE_TYPE_DATA_LL_CTRL)
        {
            /* received frame is a LL control frame */
            process_llid_control_mesg(packetbuf_dataptr());
        }
        else
        {
            /* LLID messages and fragments are handled in the mac layer */
            NETSTACK_MAC.input();
        }
    }
}

/*---------------------------------------------------------------------------*/
static void
state_conn_slave(process_event_t ev, process_data_t data,
                              uint8_t *cmd, uint8_t *param)
{
    rfc_dataEntryGeneral_t *entry;
    dataQueue_t *tx_queue = NULL;

    if(ev == rf_core_timer_event) {
        /* calculate parameters for upcoming connection event */
        conn_event.start = conn_event.next_start;
        conn_event.counter++;
        conn_event.channel = (conn_event.channel + conn_param.hop) % 37;

        /* check if tx packets or auto acks are transmitted */
        entry = (rfc_dataEntryGeneral_t *) current_tx_entry;

        /* create & send slave command for upcoming connection event */
        rf_ble_cmd_create_slave_params(param, &rx_data_queue, tx_queue,
                            conn_param.access_address, conn_param.crc_init_0,
                            conn_param.crc_init_1, conn_param.crc_init_2,
                            conn_param.win_size, CONN_EVENT_START_BEFORE_ANCHOR,
                            0);

        rf_ble_cmd_create_slave_cmd(cmd, conn_event.channel, param,
                NULL, (conn_event.start - CONN_EVENT_START_BEFORE_ANCHOR));

        if (rf_ble_cmd_send(cmd) != RF_BLE_CMD_OK) {
            PRINTF("connection error; event counter: %u\n", conn_event.counter);
            return;
        }

        /* calculate next anchor point & setup timer interrupt */
        conn_event.next_start = first_conn_event_anchor +
                (conn_param.interval * (conn_event.counter + 1));
        rf_core_start_timer_comp(conn_event.next_start -
                CONN_EVENT_WAKEUP_BEFORE_ANCHOR);
    } else if(ev == rf_core_data_rx_event) {
        process_rx_entry_data_channel();
        free_finished_rx_bufs();
    } else if(ev == rf_core_data_tx_event) {
        free_finished_tx_bufs();
    }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_controller_process, ev, data)
{
    PROCESS_BEGIN();
    PRINTF("ble_controller_process started\n");

    while(1)
    {
        PROCESS_WAIT_EVENT();
        switch(state)
        {
        case BLE_CONTROLLER_STATE_STANDBY:
            /* nothing to do here */
            break;
        case BLE_CONTROLLER_STATE_ADVERTISING:
            state_advertising(ev, data, (uint8_t *) cmd_buf,
                              (uint8_t *) param_buf);
            break;
        case BLE_CONTROLLER_STATE_SCANNING:
            // TODO
            break;
        case BLE_CONTROLLER_STATE_INITIATING:
            // TODO
            break;
        case BLE_CONTROLLER_STATE_CONN_MASTER:
            // TODO
            break;
        case BLE_CONTROLLER_STATE_CONN_SLAVE:
            state_conn_slave(ev, data, (uint8_t *) cmd_buf,
                              (uint8_t *) param_buf);
            break;
        }
    }

    PRINTF("ble_controller_process stopped\n");
    PROCESS_END();
}
