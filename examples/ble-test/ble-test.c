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

#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
PROCESS(ble_test_process, "BLE test process");
AUTOSTART_PROCESSES(&ble_test_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_test_process, ev, data)
{
	static struct etimer timer;
	PROCESS_BEGIN();

	printf("BLE test started\n");
	leds_set(LEDS_GREEN);

    while(1) {
    	etimer_set(&timer, CLOCK_CONF_SECOND);
    	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

        leds_on(LEDS_RED);
        ble_nc_advertisement();
        leds_off(LEDS_RED);
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
