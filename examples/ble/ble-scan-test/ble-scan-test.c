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
 * 		   A test for the Bluetooth Low-Energy radio of Contiki in scanning mode
 * \author
 *         Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "button-sensor.h"

#include "rf-core/ble-stack/ble-controller.h"

#include <stdio.h>
#include <string.h>

#define DEMO_INTERVAL   (10 * CLOCK_SECOND)

#define SCAN_INTERVAL   (10 * CLOCK_SECOND)
#define SCAN_WINDOW     ( 9 * CLOCK_SECOND)

static struct etimer timer;

/*---------------------------------------------------------------------------*/
PROCESS(ble_scan_test_process, "BLE scanning test process");
AUTOSTART_PROCESSES(&ble_scan_test_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_scan_test_process, ev, data)
{
    int result;
    PROCESS_BEGIN();


    printf("BLE scanning test started\n");
    leds_on(LEDS_GREEN);

    result = ble_controller_set_scan_parameters(SCAN_INTERVAL, SCAN_WINDOW, 37);

    while(1) {
        printf("BLE scanning: while loop\n");
        ble_controller_set_scan_enable();
        etimer_set(&timer, DEMO_INTERVAL);
        PROCESS_YIELD_UNTIL(etimer_expired(&timer));
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
