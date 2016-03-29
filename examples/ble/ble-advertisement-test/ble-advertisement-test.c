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
/**
 * \file
 * 		   A test for the Bluetooth Low-Energy radio of Contiki
 * \author
 *         Michael Spörk <m.spoerk@student.tugraz.at>
 */

#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "button-sensor.h"
#include "rf-core/ble-stack/ble-controller.h"

#include <stdio.h>
#include <string.h>

#define BLE_TEST_INTERVAL (CLOCK_SECOND * 60)
#define BLE_PAUSE_INTERVAL  (CLOCK_SECOND * 1)
#define BLE_ADVERTISEMENT_BUFFER_LENGTH 31
#define BLE_SCAN_RESPONSE_BUFFER_LENGTH 31
#define BLE_ADVERTISEMENT_DEVICE_NAME "TI SensorTag"

static struct etimer timer;
static char adv_data[BLE_ADVERTISEMENT_BUFFER_LENGTH];
static int adv_data_len;
static char scan_resp_data[BLE_SCAN_RESPONSE_BUFFER_LENGTH];
static int scan_resp_data_len;

/*---------------------------------------------------------------------------*/
PROCESS(ble_test_process, "BLE advertising test process");
AUTOSTART_PROCESSES(&ble_test_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_test_process, ev, data)
{
	PROCESS_BEGIN();

//	/* initialize advertisement data */
//	memset(adv_data, 0, BLE_ADVERTISEMENT_BUFFER_LENGTH);
//	adv_data[adv_data_len++] = 0x02;
//	adv_data[adv_data_len++] = 0x01;	// BLE device info
//	adv_data[adv_data_len++] = 0x1a;	// LE general discoverable + BR/EDR
//	adv_data[adv_data_len++] = 1 + strlen(BLE_ADVERTISEMENT_DEVICE_NAME);
//	adv_data[adv_data_len++] = 0x09;	// BLE device name
//	memcpy(&adv_data[adv_data_len], BLE_ADVERTISEMENT_DEVICE_NAME,
//	       strlen(BLE_ADVERTISEMENT_DEVICE_NAME));
//	adv_data_len += strlen(BLE_ADVERTISEMENT_DEVICE_NAME);
//
//	/* initialize scan response data */
//	memset(scan_resp_data, 0, BLE_SCAN_RESPONSE_BUFFER_LENGTH);
//	for(scan_resp_data_len = 0; scan_resp_data_len < 16; ++scan_resp_data_len){
//	    scan_resp_data[scan_resp_data_len] = scan_resp_data_len;
//	}
//


	/* initialize advertising data */
	memset(adv_data , 0x00, BLE_ADVERTISEMENT_BUFFER_LENGTH);
	/* FLAGS */
	adv_data[adv_data_len++] = 2;
	adv_data[adv_data_len++] = 0x01;
	adv_data[adv_data_len++] = 0x06;        // LE general discoverable (no BR/EDR support)
	/* TX power level */
	adv_data[adv_data_len++] = 2;           // 2 bytes long
	adv_data[adv_data_len++] = 0x0A;
	adv_data[adv_data_len++] = 0;           // 0 dBm TODO: get actual value
	/* service UUIDs (16-bit) */
	adv_data[adv_data_len++] = 3;
	adv_data[adv_data_len++] = 0x03;
	adv_data[adv_data_len++] = 0x20;        // UUID of IPSS
	adv_data[adv_data_len++] = 0x18;
	/* service UUIDs (32-bit) */
	adv_data[adv_data_len++] = 1;
	adv_data[adv_data_len++] = 0x05;
    /* service UUIDs (128-bit) */
    adv_data[adv_data_len++] = 1;
    adv_data[adv_data_len++] = 0x07;

    /* initialize scan response data */
    memset(scan_resp_data, 0x00, BLE_SCAN_RESPONSE_BUFFER_LENGTH);
    /* complete device name */
    scan_resp_data[scan_resp_data_len++] = 1 + strlen(BLE_ADVERTISEMENT_DEVICE_NAME);
    scan_resp_data[scan_resp_data_len++] = 0x09;
    memcpy(&scan_resp_data[scan_resp_data_len], BLE_ADVERTISEMENT_DEVICE_NAME, strlen(BLE_ADVERTISEMENT_DEVICE_NAME));
    scan_resp_data_len += strlen(BLE_ADVERTISEMENT_DEVICE_NAME);
    /* slave connection interval range */
    scan_resp_data[scan_resp_data_len++] = 5;
    scan_resp_data[scan_resp_data_len++] = 0x12;
    scan_resp_data[scan_resp_data_len++] = 0x06;
    scan_resp_data[scan_resp_data_len++] = 0x00;
    scan_resp_data[scan_resp_data_len++] = 0x80;
    scan_resp_data[scan_resp_data_len++] = 0x0C;


	printf("BLE advertisement test\n");
	leds_set(LEDS_GREEN);

	ble_controller_set_advertising_parameters((CLOCK_SECOND * 1), 0b0111);
	ble_controller_set_advertising_data(adv_data_len, adv_data);
	ble_controller_set_scan_response_data(scan_resp_data_len, scan_resp_data);

    while(1) {
        ble_controller_enable_advertising();
        printf("test: advertising started\n");
        etimer_set(&timer, BLE_TEST_INTERVAL);
        PROCESS_YIELD_UNTIL(etimer_expired(&timer));
        ble_controller_disable_advertising();
        printf("test: advertising stopped\n");
        etimer_set(&timer, BLE_PAUSE_INTERVAL);
        PROCESS_YIELD_UNTIL(etimer_expired(&timer));
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
