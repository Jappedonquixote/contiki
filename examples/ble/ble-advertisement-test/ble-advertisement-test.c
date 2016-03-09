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
#include "rf-core/ble-utils.h"

#include <stdio.h>
#include <string.h>

#define BLE_ADVERTISEMENT_TIMEOUT (CLOCK_CONF_SECOND * 10)
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
	int adv_channel;
	PROCESS_BEGIN();

	/* initialize advertisement data */
	memset(adv_data, 0, BLE_ADVERTISEMENT_BUFFER_LENGTH);
	adv_data[adv_data_len++] = 0x02;
	adv_data[adv_data_len++] = 0x01;	// BLE device info
	adv_data[adv_data_len++] = 0x1a;	// LE general discoverable + BR/EDR
	adv_data[adv_data_len++] = 1 + strlen(BLE_ADVERTISEMENT_DEVICE_NAME);
	adv_data[adv_data_len++] = 0x09;	// BLE device name
	memcpy(&adv_data[adv_data_len], BLE_ADVERTISEMENT_DEVICE_NAME,
	       strlen(BLE_ADVERTISEMENT_DEVICE_NAME));
	adv_data_len += strlen(BLE_ADVERTISEMENT_DEVICE_NAME);

	/* initialize scan response data */
	memset(scan_resp_data, 0, BLE_SCAN_RESPONSE_BUFFER_LENGTH);
	for(scan_resp_data_len = 0; scan_resp_data_len < 16; ++scan_resp_data_len){
	    scan_resp_data[scan_resp_data_len] = scan_resp_data_len;
	}

	printf("BLE advertisement test\n");
	leds_set(LEDS_GREEN);

    while(1) {
        leds_on(LEDS_RED);
        for(adv_channel = 37; adv_channel <= 39; ++adv_channel) {
        	ble_utils_send_advertisement(adv_channel,
        	                             adv_data, adv_data_len,
        	                             scan_resp_data, scan_resp_data_len);
        }
        leds_off(LEDS_RED);

        etimer_set(&timer, BLE_ADVERTISEMENT_TIMEOUT);
        PROCESS_YIELD_UNTIL(etimer_expired(&timer));
        ble_utils_print_advertisement_output_data();
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
