/**
 * \file
 * 		   A test for the Bluetooth Low-Energy radio of Contiki
 * \author
 *         Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "button-sensor.h"
#include "net/ble.h"

#include <stdio.h>
#include <string.h>

#define BLE_ADVERTISEMENT_TIMEOUT (CLOCK_CONF_SECOND * 10)
#define BLE_ADVERTISEMENT_BUFFER_LENGTH 64
#define BLE_ADVERTISEMENT_DEVICE_NAME "TI SensorTag"

static struct etimer timer;
static char payload[BLE_ADVERTISEMENT_BUFFER_LENGTH];
static int payload_len;

/*---------------------------------------------------------------------------*/
PROCESS(ble_test_process, "BLE test process");
AUTOSTART_PROCESSES(&ble_test_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_test_process, ev, data)
{

//	memset(payload, 0, BLE_ADV_PAYLOAD_BUF_LEN);
//	payload[p++] = 0x02;          /* 2 bytes */
//	    payload[p++] = BLE_ADV_TYPE_DEVINFO;
//	    payload[p++] = 0x1a;          /* LE general discoverable + BR/EDR */
//	    payload[p++] = 1 + strlen(beacond_config.adv_name);
//	    payload[p++] = BLE_ADV_TYPE_NAME;
//	    memcpy(&payload[p], beacond_config.adv_name,
//	           strlen(beacond_config.adv_name));
//	    p += strlen(beacond_config.adv_name);

	int adv_channel;
	PROCESS_BEGIN();

	memset(payload, 0, BLE_ADVERTISEMENT_BUFFER_LENGTH);
	payload[payload_len++] = 0x02;
	payload[payload_len++] = 0x01;	// BLE device info
	payload[payload_len++] = 0x1a;	// LE general discoverable + BR/EDR
	payload[payload_len++] = 1 + strlen(BLE_ADVERTISEMENT_DEVICE_NAME);
	payload[payload_len++] = 0x09;	// BLE device name
	memcpy(&payload[payload_len], BLE_ADVERTISEMENT_DEVICE_NAME, strlen(BLE_ADVERTISEMENT_DEVICE_NAME));
	payload_len += strlen(BLE_ADVERTISEMENT_DEVICE_NAME);


	printf("BLE test started\n");
	leds_set(LEDS_GREEN);

    while(1) {
    	etimer_set(&timer, BLE_ADVERTISEMENT_TIMEOUT);
    	PROCESS_YIELD_UNTIL(etimer_expired(&timer));

    	printf("advertising: %s / %d\n", payload, payload_len);

        leds_on(LEDS_RED);
        for(adv_channel = 37; adv_channel <= 39; ++adv_channel) {
        	ble_send_advertisement(adv_channel, payload, payload_len);
        }
        leds_off(LEDS_RED);
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
