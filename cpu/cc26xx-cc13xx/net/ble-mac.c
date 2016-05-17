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
 * ble-mac.c
 *
 *      Author: Michael Spörk
 */

#include "net/ble-mac.h"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include "net/frame-ble.h"

#include "dev/ble-controller.h"

#include <string.h>

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTADDR(addr) PRINTF("%02X:%02X:%02X:%02X:%02X:%02X", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5])
#else
#define PRINTF(...)
#define PRINTADDR(addr)
#endif
/*---------------------------------------------------------------------------*/
#define BLE_DEVICE_NAME "TI Sensortag"
#define BLE_SLAVE_CONN_INTERVAL_MIN  0x0150
#define BLE_SLAVE_CONN_INTERVAL_MAX  0x01F0

#define BLE_MAC_L2CAP_SIGNAL_CHANNEL 0x0005
#define BLE_MAC_L2CAP_FLOW_CHANNEL   0xCAFE

#define BLE_MAC_L2CAP_CODE_CONN_REQ    0x14
#define BLE_MAC_L2CAP_CODE_CONN_RSP    0x15
#define BLE_MAC_L2CAP_CODE_CREDIT      0x16

#define BLE_MAC_L2CAP_NODE_MTU         PACKETBUF_SIZE
#define BLE_MAC_L2CAP_NODE_MPS         1280
#define BLE_MAC_L2CAP_NODE_INIT_CREDITS  10
#define BLE_MAC_L2CAP_CREDIT_THRESHOLD    2

#define BLE_MAC_L2CAP_HEADER_SIZE         4
/*---------------------------------------------------------------------------*/
/* BLE controller */
/* public device address of BLE controller */
static uint8_t ble_addr[BLE_ADDR_SIZE];

/* length of a single BLE controller buffer */
static int buffer_len;
/* Number of buffers available at the BLE controller */
static int num_buffer;
/*---------------------------------------------------------------------------*/
typedef struct {
    uint16_t cid;
    uint16_t mtu;
    uint16_t mps;
    uint16_t credits;
} ble_mac_l2cap_channel_t;

static ble_mac_l2cap_channel_t l2cap_router;
static ble_mac_l2cap_channel_t l2cap_node;
/*---------------------------------------------------------------------------*/
static uint8_t init_adv_data(char *adv_data)
{
    uint8_t adv_data_len = 0;
    memset(adv_data, 0x00, BLE_ADV_DATA_LEN);
    /* BLE flags */
    adv_data[adv_data_len++] = 2;
    adv_data[adv_data_len++] = 0x01;
    adv_data[adv_data_len++] = 0x05;   /* LE limited  (no BR/EDR support) */
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
    return adv_data_len;
}
/*---------------------------------------------------------------------------*/
static uint8_t init_scan_resp_data(char *scan_resp_data)
{
    uint8_t scan_resp_data_len = 0;
    memset(scan_resp_data, 0x00, BLE_SCAN_RESP_DATA_LEN);
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

    return scan_resp_data_len;
}
/*---------------------------------------------------------------------------*/
static void init(void)
{
    uint8_t adv_data_len, scan_resp_data_len;
    char adv_data[BLE_ADV_DATA_LEN];
    char scan_resp_data[BLE_SCAN_RESP_DATA_LEN];

    PRINTF("[ ble-mac ] init()\n");

    /* initialize the L2CAP connection parameter */
    l2cap_node.cid = BLE_MAC_L2CAP_FLOW_CHANNEL;
    l2cap_node.credits = BLE_MAC_L2CAP_NODE_INIT_CREDITS;
    l2cap_node.mps = BLE_MAC_L2CAP_NODE_MPS;
    l2cap_node.mtu = BLE_MAC_L2CAP_NODE_MTU;

    /* Initialize the BLE controller */
    NETSTACK_RADIO.init();
    NETSTACK_RADIO.get_object(RADIO_CONST_BLE_BD_ADDR, &ble_addr, BLE_ADDR_SIZE);
    NETSTACK_RADIO.get_value(RADIO_CONST_BLE_BUFFER_SIZE, &buffer_len);
    NETSTACK_RADIO.get_value(RADIO_CONST_BLE_BUFFER_AMOUNT, &num_buffer);

    PRINTF("ble-mac init() BLE-addr: ");
    PRINTADDR(ble_addr);
    PRINTF("; buffer-len: %d; num-buffer: %d\n", buffer_len, num_buffer);


    /* set the advertisement parameter */
    NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_INTERVAL, 0x0800);
    NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_TYPE, BLE_ADV_DIR_IND_LDC);
    NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_OWN_ADDR_TYPE, BLE_ADDR_TYPE_PUBLIC);
    NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_CHANNEL_MAP, 0x01);

    adv_data_len = init_adv_data(adv_data);
    scan_resp_data_len = init_scan_resp_data(scan_resp_data);

    /* set advertisement payload & scan response */
    NETSTACK_RADIO.set_object(RADIO_PARAM_BLE_ADV_PAYLOAD, adv_data, adv_data_len);
    NETSTACK_RADIO.set_object(RADIO_PARAM_BLE_ADV_SCAN_RESPONSE, scan_resp_data, scan_resp_data_len);

    /* enable advertisement */
    NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_ENABLE, 1);
}
/*---------------------------------------------------------------------------*/
static void prepare_l2cap_header(void) {

    uint16_t sdu_len = packetbuf_datalen();
    uint16_t mtu_len = sdu_len + 2;

    void *hdr_ptr;

    // packet length + channel + sdu length
    packetbuf_hdralloc(6);
    hdr_ptr = packetbuf_hdrptr();

    memset(hdr_ptr, 0x00, 6);

    memcpy(hdr_ptr, &mtu_len, 2);
    memcpy(&hdr_ptr[2], &l2cap_router.cid, 2);
    memcpy(&hdr_ptr[4], &sdu_len, 2);
}

/*---------------------------------------------------------------------------*/
static void send(mac_callback_t sent_callback, void *ptr)
{

    PRINTF("[ ble-mac ] send() len: %d\n", packetbuf_datalen());
    prepare_l2cap_header();

    if(l2cap_router.credits <= 0) {
        PRINTF("ble-mac send() no more credits left\n");
    }

    l2cap_router.credits--;
    NETSTACK_RDC.send(sent_callback, ptr);
}

/*---------------------------------------------------------------------------*/
void process_l2cap_conn_req(uint8_t *data)
{
    uint8_t identifier;
    uint16_t len;
    uint16_t le_psm;
    uint8_t resp_data[26];

    identifier = data[0];
    memcpy(&len, &data[1], 2);

    if(len != 10)
    {
        PRINTF("process_l2cap_conn_req: invalid len: %d\n", len);
        return;
    }

    /* parse L2CAP connection data */
    memcpy(&le_psm, &data[3], 2);
    memcpy(&l2cap_router.cid, &data[5], 2);
    memcpy(&l2cap_router.mtu, &data[7], 2);
    memcpy(&l2cap_router.mps, &data[9], 2);
    memcpy(&l2cap_router.credits, &data[11], 2);

    /* create L2CAP connection response */
    /* length */
    resp_data[0] = 0x0E;
    resp_data[1] = 0x00;

    /* channel ID */
    resp_data[2] = 0x05;
    resp_data[3] = 0x00;

    /* code */
    resp_data[4] = BLE_MAC_L2CAP_CODE_CONN_RSP;

    /* identifier */
    resp_data[5] = identifier;

    /* cmd length */
    resp_data[6] = 0x0A;
    resp_data[7] = 0x00;

    /* node channel information */
    memcpy(&resp_data[8], &l2cap_node.cid, 2);
    memcpy(&resp_data[10], &l2cap_node.mtu, 2);
    memcpy(&resp_data[12], &l2cap_node.mps, 2);
    memcpy(&resp_data[14], &l2cap_node.credits, 2);

    /* result */
    memset(&resp_data[16], 0x00, 2);

    packetbuf_copyfrom((void *) resp_data, 18);
   NETSTACK_RDC.send(NULL, NULL);

}

/*---------------------------------------------------------------------------*/
void process_l2cap_credit(uint8_t *data)
{
    uint8_t identifier;
    uint16_t len;
    uint16_t cid;
    uint16_t credits;

    identifier = data[0];
    memcpy(&len, &data[1], 2);

    if(len != 4)
    {
        PRINTF("process_l2cap_credit: invalid len: %d\n", len);
        return;
    }

    /* parse L2CAP credit data */
    memcpy(&cid, &data[3], 2);
    memcpy(&credits, &data[5], 2);

    l2cap_router.credits += credits;

    PRINTF("process_l2cap_credit() new credits: %d\n", l2cap_router.credits);
}

/*---------------------------------------------------------------------------*/
static void process_l2cap_frame_signal_channel(uint8_t *data, uint8_t data_len)
{
    if (data[4] == BLE_MAC_L2CAP_CODE_CONN_REQ) {
        process_l2cap_conn_req(&data[5]);
    } else if(data[4] == BLE_MAC_L2CAP_CODE_CREDIT) {
        PRINTF("credit received\n");
        process_l2cap_credit(&data[5]);
    } else {
        PRINTF("process_l2cap_msg: unknown signal channel code: %d\n", data[4]);
    }
}



/*---------------------------------------------------------------------------*/
static void process_l2cap_frame_flow_channel(uint8_t *data, uint8_t data_len)
{
    uint16_t len;
    uint16_t sdu_len;

    if(data_len < 4) {
        PRINTF("process_l2cap_frame: illegal L2CAP frame data_len: %d\n", data_len);
        /* a L2CAP frame has a minimum length of 4 */
        return;
    }

    memcpy(&len, &data[0], 2);
    memcpy(&sdu_len, &data[4], 2);

    if(sdu_len > (len - 2)) {
        /* the L2CAP message is fragmented */
        PRINTF("process_l2cap_frame_flow_channel: fragmented message\n");
    } else {
        /* remove the L2CAP header from the packetbuffer */
        packetbuf_hdrreduce(6);
        NETSTACK_NETWORK.input();
    }

}
/*---------------------------------------------------------------------------*/
static void send_l2cap_credit()
{
    uint8_t len = 4;
    uint8_t data[12];

    /* create L2CAP credit */
    /* length */
    data[0] = len + 4;
    data[1] = 0x00;

    /* channel ID */
    data[2] = 0x05;
    data[3] = 0x00;

    /* code */
    data[4] = BLE_MAC_L2CAP_CODE_CREDIT;
    /* identifier */
    data[5] = 0xFF;
    /* cmd length */
    data[6] = len;
    data[7] = 0x00;

    memcpy(&data[8], &l2cap_node.cid, 2);
    data[10] = BLE_MAC_L2CAP_NODE_INIT_CREDITS & 0xFF;
    data[11] = BLE_MAC_L2CAP_NODE_INIT_CREDITS >> 8;

    packetbuf_copyfrom((void *) data, len + 8);
    NETSTACK_RDC.send(NULL, NULL);
}

/*---------------------------------------------------------------------------*/
static void input(void)
{
    uint8_t *data = (uint8_t *) packetbuf_dataptr();
    uint8_t len = packetbuf_datalen();
    uint16_t channel_id;

    memcpy(&channel_id, &data[2], 2);

    if(len > 0) {
        if(channel_id == BLE_MAC_L2CAP_SIGNAL_CHANNEL) {
            process_l2cap_frame_signal_channel(data, len);
        }
        else if(channel_id == BLE_MAC_L2CAP_FLOW_CHANNEL) {
            process_l2cap_frame_flow_channel(data, len);
            /* decrease the credits of the router */
            l2cap_node.credits--;
            if(l2cap_node.credits <= BLE_MAC_L2CAP_CREDIT_THRESHOLD) {
                PRINTF("ble-mac input() sending credit\n");
                send_l2cap_credit();
                l2cap_node.credits += BLE_MAC_L2CAP_NODE_INIT_CREDITS;
            }
        }
        else {
            PRINTF("ble-mac input: unknown L2CAP channel: %x\n", channel_id);
            return;
        }
    }
}

/*---------------------------------------------------------------------------*/
static int on(void)
{
    PRINTF("[ ble-mac ] on()\n");
    return NETSTACK_RDC.on();
}

/*---------------------------------------------------------------------------*/
static int off(int keep_radio_on)
{
    PRINTF("[ ble-mac ] off()\n");
    return NETSTACK_RDC.off(keep_radio_on);
}

/*---------------------------------------------------------------------------*/
static unsigned short channel_check_interval(void)
{
    return 0;
}

/*---------------------------------------------------------------------------*/
const struct mac_driver ble_mac_driver = {
  "ble_mac_connection_based",
  init,
  send,
  input,
  on,
  off,
  channel_check_interval,
};
