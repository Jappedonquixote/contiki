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

#include <string.h>

/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static unsigned int adv_data_length = 0;
static unsigned int scan_resp_data_length = 0;
static char adv_data[BLE_ADV_DATA_LENGHT_MAX];
static char scan_resp_data[BLE_ADV_DATA_LENGHT_MAX];

static unsigned short connection_established = 0;

/*---------------------------------------------------------------------------*/
static void init_advertising_data()
{
    adv_data_length = 0;
    memset(adv_data, 0x00, BLE_ADV_DATA_LENGHT_MAX);
    /* BLE flags */
    adv_data[adv_data_length++] = 2;
    adv_data[adv_data_length++] = 0x01;
    adv_data[adv_data_length++] = 0x05;   /* LE limited discoverable (no BR/EDR support) */
    /* TX power level */
    adv_data[adv_data_length++] = 2;
    adv_data[adv_data_length++] = 0x0A;
    adv_data[adv_data_length++] = 0;      /* 0 dBm; TODO: get actual tx power value */
    /* service UUIDs (16-bit identifiers) */
    adv_data[adv_data_length++] = 3;
    adv_data[adv_data_length++] = 0x03;
    adv_data[adv_data_length++] = 0x20;
    adv_data[adv_data_length++] = 0x18;   /* only IP support service exposed */
    /* service UUIDs (32-bit identifiers) */
    adv_data[adv_data_length++] = 1;
    adv_data[adv_data_length++] = 0x05;   /* empty list */
    /* service UUIDs (128-bit identifiers) */
    adv_data[adv_data_length++] = 1;
    adv_data[adv_data_length++] = 0x07;   /* empty list */
}
/*---------------------------------------------------------------------------*/
static void init_scan_resp_data()
{
    scan_resp_data_length = 0;
    memset(scan_resp_data, 0x00, BLE_ADV_DATA_LENGHT_MAX);
    /* complete device name */
    scan_resp_data[scan_resp_data_length++] = 1 + strlen(BLE_DEVICE_NAME);
    scan_resp_data[scan_resp_data_length++] = 0x09;
    memcpy(&scan_resp_data[scan_resp_data_length],
           BLE_DEVICE_NAME, strlen(BLE_DEVICE_NAME));
    scan_resp_data_length += strlen(BLE_DEVICE_NAME);
    /* slave connection interval range */
    scan_resp_data[scan_resp_data_length++] = 5;
    scan_resp_data[scan_resp_data_length++] = 0x12;
    scan_resp_data[scan_resp_data_length++] = (BLE_SLAVE_CONN_INTERVAL_MIN & 0xFF);
    scan_resp_data[scan_resp_data_length++] = ((BLE_SLAVE_CONN_INTERVAL_MIN >> 8) & 0xFF);
    scan_resp_data[scan_resp_data_length++] = (BLE_SLAVE_CONN_INTERVAL_MAX & 0xFF);
    scan_resp_data[scan_resp_data_length++] = ((BLE_SLAVE_CONN_INTERVAL_MAX >> 8) & 0xFF);
}
/*---------------------------------------------------------------------------*/
static void init(void)
{
    int result;
    PRINTF("[ ble-mac ] init()\n");

    init_advertising_data();
    init_scan_resp_data();

    result = ble_controller_set_advertising_parameters(BLE_ADV_INTERVAL,
                                                       BLE_ADV_CHANNEL_MASK);
    if(result != BLE_COMMAND_SUCCESS)
    {
        PRINTF("[ ble-mac ] init() could not set adv parameters\n");
        return;
    }
    result = ble_controller_set_advertising_data(adv_data_length, adv_data);
    if(result != BLE_COMMAND_SUCCESS)
    {
        PRINTF("[ ble-mac ] init() could not set adv data\n");
        return;
    }
    result = ble_controller_set_scan_response_data(scan_resp_data_length, scan_resp_data);
    if(result != BLE_COMMAND_SUCCESS)
    {
        PRINTF("[ ble-mac ] init() could not set scan resp data\n");
        return;
    }
    result = ble_controller_enable_advertising();
    if(result != BLE_COMMAND_SUCCESS)
    {
        PRINTF("[ ble-mac ] init() could not enable advertising\n");
        return;
    }
    PRINTF("[ ble-mac ] init() advertising started\n");
}

/*---------------------------------------------------------------------------*/
static void send(mac_callback_t sent_callback, void *ptr)
{
    PRINTF("[ ble-mac ] send()\n");
    NETSTACK_RDC.send(sent_callback, ptr);
}


/*---------------------------------------------------------------------------*/
static void input(void)
{
    PRINTF("[ ble-mac ] input()\n");
    if((connection_established == 0) &&
       (packetbuf_attr(PACKETBUF_ATTR_FRAME_TYPE) == FRAME_BLE_ADV_PDU_CONNECT_REQ))
    {
        PRINTF("[ ble-mac ] input() CONNECT REQ received\n");
    }
//    NETSTACK_LLSEC.input();
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
