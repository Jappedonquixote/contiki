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
/**
 * \file
 *       A test for the Bluetooth Low-Energy radio of Contiki
 *       This test shows how to start advertisements on the BLE controller
 * \author
 *         Michael Spoerk <m.spoerk@student.tugraz.at>
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "dev/leds.h"
#include "ble-hal.h"
#include "contiki-net.h"

#include <stdio.h>

/*---------------------------------------------------------------------------*/
/* BLE advertisement constants */
#define BLE_DEVICE_NAME "TI Sensortag - Contiki"
#define BLE_SLAVE_CONN_INTERVAL_MIN  0x0150
#define BLE_SLAVE_CONN_INTERVAL_MAX  0x01F0

#define L2CAP_SIGNAL_CHANNEL 0x0005
#define L2CAP_FLOW_CHANNEL   0x0041

#define L2CAP_CODE_CONN_REQ    0x14
#define L2CAP_CODE_CONN_RSP    0x15
#define L2CAP_CODE_CREDIT      0x16

#define L2CAP_NODE_MTU         1280
#define L2CAP_NODE_FRAG_LEN     160
#define L2CAP_NODE_INIT_CREDITS   8
#define L2CAP_CREDIT_THRESHOLD    2

#define L2CAP_FIRST_HEADER_SIZE         6
#define L2CAP_FIRST_FRAGMENT_SIZE   (L2CAP_NODE_FRAG_LEN - L2CAP_FIRST_HEADER_SIZE)
#define L2CAP_SUBSEQ_HEADER_SIZE        4
#define L2CAP_SUBSEQ_FRAGMENT_SIZE  (L2CAP_NODE_FRAG_LEN - L2CAP_SUBSEQ_HEADER_SIZE)
#define L2CAP_TRANSMISSION_DELAY    (CLOCK_SECOND / 32)

/*---------------------------------------------------------------------------*/
static uint8_t
init_adv_data(char *adv_data)
{
  uint8_t adv_data_len = 0;
  memset(adv_data, 0x00, BLE_ADV_DATA_LEN);
  /* BLE flags */
  adv_data[adv_data_len++] = 2;
  adv_data[adv_data_len++] = 0x01;
  adv_data[adv_data_len++] = 0x05;     /* LE limited  (no BR/EDR support) */
  /* TX power level */
  adv_data[adv_data_len++] = 2;
  adv_data[adv_data_len++] = 0x0A;
  adv_data[adv_data_len++] = 0;        /* 0 dBm */
  /* service UUIDs (16-bit identifiers) */
  adv_data[adv_data_len++] = 3;
  adv_data[adv_data_len++] = 0x03;
  adv_data[adv_data_len++] = 0x20;
  adv_data[adv_data_len++] = 0x18;     /* only IP support service exposed */
  /* service UUIDs (32-bit identifiers) */
  adv_data[adv_data_len++] = 1;
  adv_data[adv_data_len++] = 0x05;     /* empty list */
  /* service UUIDs (128-bit identifiers) */
  adv_data[adv_data_len++] = 1;
  adv_data[adv_data_len++] = 0x07;     /* empty list */
  return adv_data_len;
}
/*---------------------------------------------------------------------------*/
static uint8_t
init_scan_resp_data(char *scan_resp_data)
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
PROCESS(ble_hal_demo_process, "BLE HAL demo process");
AUTOSTART_PROCESSES(&ble_hal_demo_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_hal_demo_process, ev, data)
{
    uint8_t adv_data_len, scan_resp_data_len;
    char adv_data[BLE_ADV_DATA_LEN];
    char scan_resp_data[BLE_SCAN_RESP_DATA_LEN];

  PROCESS_BEGIN();
  leds_on(LEDS_GREEN);

  printf("BLE HAL demo process started\n");

  printf("starting BLE advertisement\n");

  // for the values used see the Bluetooth Specification  4.1 (p 1240ff)
  // set advertisement parameter
  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_INTERVAL, 0x0800);
  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_TYPE, BLE_ADV_DIR_IND_LDC);
  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_OWN_ADDR_TYPE, BLE_ADDR_TYPE_PUBLIC);
  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_CHANNEL_MAP, 0x01);

  // set advertisement data
  adv_data_len = init_adv_data(adv_data);
  NETSTACK_RADIO.set_object(RADIO_PARAM_BLE_ADV_PAYLOAD, adv_data, adv_data_len);

  // set scan response data
  scan_resp_data_len = init_scan_resp_data(scan_resp_data);
  NETSTACK_RADIO.set_object(RADIO_PARAM_BLE_ADV_SCAN_RESPONSE, scan_resp_data, scan_resp_data_len);

  /* enable advertisement */
  NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_ADV_ENABLE, 1);

  while(1) {
      leds_on(LEDS_RED);
      PROCESS_YIELD();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
