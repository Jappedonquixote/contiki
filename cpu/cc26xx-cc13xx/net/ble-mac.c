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
#include "net/ble-rdc.h"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include "net/frame-ble.h"

#include "rf-core/ble-stack/ble-radio-controller.h"

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
#define BLE_MAC_L2CAP_SIGNAL_CHANNEL 0x0005
#define BLE_MAC_L2CAP_FLOW_CHANNEL   0x0069

#define BLE_MAC_L2CAP_CODE_CONN_REQ    0x14
#define BLE_MAC_L2CAP_CODE_CONN_RSP    0x15

#define BLE_MAC_L2CAP_NODE_MTU       0xFFFF
#define BLE_MAC_L2CAP_NODE_MPS       0x00FF
#define BLE_MAC_L2CAP_NODE_INIT_CREDITS  10

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
void print_l2cap_channel(ble_mac_l2cap_channel_t channel)
{
    PRINTF("L2CAP channel information\n");
    PRINTF("CID:    0x%04X\n", channel.cid);
    PRINTF("MTU:     %5d\n", channel.mtu);
    PRINTF("MPS:     %5d\n", channel.mps);
    PRINTF("credits: %5d\n", channel.credits);
}
/*---------------------------------------------------------------------------*/
static void init(void)
{
    PRINTF("[ ble-mac ] init()\n");

    /* initialize the l2cap channel information */
    l2cap_node.cid = BLE_MAC_L2CAP_FLOW_CHANNEL;
    l2cap_node.mtu = BLE_MAC_L2CAP_NODE_MTU;
    l2cap_node.mps = BLE_MAC_L2CAP_NODE_MPS;
    l2cap_node.credits = BLE_MAC_L2CAP_NODE_INIT_CREDITS;
}

/*---------------------------------------------------------------------------*/
static void send(mac_callback_t sent_callback, void *ptr)
{
//    PRINTF("[ ble-mac ] send()\n");
//    NETSTACK_RDC.send(sent_callback, ptr);
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

    PRINTF("L2CAP connection request:\n");
    print_l2cap_channel(l2cap_router);

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
void process_l2cap_frame(uint8_t *data, uint8_t data_len)
{
    uint16_t len;
    uint16_t channel_id;

    if(data_len < 4)
    {
        PRINTF("process_l2cap_frame: illegal L2CAP frame len: %d\n", data_len);
        /* a L2CAP frame has a minimum length of 4 */
        return;
    }

    memcpy(&len, &data[0], 2);
    memcpy(&channel_id, &data[2], 2);

    PRINTF("process_l2cap_frame: len: %d; channel_id: 0x%04X\n", len, channel_id);

    if(channel_id == BLE_MAC_L2CAP_SIGNAL_CHANNEL)
    {
        if(data[4] == BLE_MAC_L2CAP_CODE_CONN_REQ)
        {
            process_l2cap_conn_req(&data[5]);
        }
    }
}

/*---------------------------------------------------------------------------*/
static void input(void)
{
    uint8_t *data = (uint8_t *) packetbuf_dataptr();
    uint8_t len = packetbuf_datalen();

    if(len > 0)
    {
        process_l2cap_frame(data, len);
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
    PRINTF("[ ble-mac ] channel_check_interval()\n");
    return 0;
}

/*---------------------------------------------------------------------------*/
const struct mac_driver ble_mac_driver = {
  "ble_mac",
  init,
  send,
  input,
  on,
  off,
  channel_check_interval,
};
