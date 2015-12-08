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

#define BLE_ADVERTISEMENT_TIMEOUT (CLOCK_CONF_SECOND * 30)
#define BLE_ADVERTISEMENT_CHANNEL 37
#define BLE_ADVERTISEMENT_PAYLOAD "payload"

/*---------------------------------------------------------------------------*/
PROCESS(ble_test_process, "BLE test process");
AUTOSTART_PROCESSES(&ble_test_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_test_process, ev, data)
{
	static struct etimer timer;
	int adv_channel = BLE_ADVERTISEMENT_CHANNEL;
	char adv_payload[] = BLE_ADVERTISEMENT_PAYLOAD;
	int adv_payload_len = strlen(adv_payload);
	PROCESS_BEGIN();

	printf("BLE test started\n");
	leds_set(LEDS_GREEN);

    while(1) {
    	etimer_set(&timer, BLE_ADVERTISEMENT_TIMEOUT);
    	PROCESS_YIELD_UNTIL(etimer_expired(&timer));

        leds_on(LEDS_RED);
        ble_send_advertisement(adv_channel, adv_payload, adv_payload_len);
        leds_off(LEDS_RED);
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
