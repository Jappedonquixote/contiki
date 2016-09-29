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
 * \author
 *         Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uiplib.h"
#include "net/ipv6/uip-icmp6.h"
#include "dev/leds.h"
#include "ti-lib.h"

#include <string.h>
#include <stdbool.h>

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define CLIENT_PORT 61617
/* nRF52 */
//#define SERVER_IP   "2001:db8::25a:11ff:fe62:4f61"
/* border router */
#define SERVER_IP   "fe80::21a:7dff:feda:7114"
#define SERVER_PORT 61616

#define ECHO_TIMEOUT        (CLOCK_SECOND * 1)
#define SEND_INTERVAL       (CLOCK_SECOND * 5)
#define MAX_PAYLOAD_LEN     1280

#define IPV6_OVERHEAD_LEN   48
#define TEST_PAYLOAD_LEN    1280

static struct etimer timer;

static struct uip_icmp6_echo_reply_notification router_notification;
static uint8_t echo_received;

static uip_ipaddr_t server_addr;
static struct uip_udp_conn *conn;

static uint32_t packet_counter = 0;

#define PACKET_OFFSET        2
#define PACKET_COUNT         20
static uint32_t latency[PACKET_COUNT];
/*---------------------------------------------------------------------------*/
PROCESS(ble_client_process, "BLE UDP client process");
AUTOSTART_PROCESSES(&ble_client_process);
/*---------------------------------------------------------------------------*/
void
echo_reply_handler(uip_ipaddr_t *source, uint8_t ttl, uint8_t *data, uint16_t datalen)
{
  printf("echo response received\n");
  echo_received = 1;
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *str;

  if(uip_newdata()) {
    ti_lib_gpio_pin_write(BOARD_IOID_DP0, 0);
    str = uip_appdata;
    /* only show the start of the packet */
    str[9] = '\0';
    printf("udp data received: %s\n", str);
    if(packet_counter >= PACKET_OFFSET) {
        latency[packet_counter - PACKET_OFFSET] = RTIMER_NOW() - latency[packet_counter - PACKET_OFFSET];
    }
  }
}
/*---------------------------------------------------------------------------*/
static char buf[MAX_PAYLOAD_LEN];
static void
timeout_handler(void)
{
    uint16_t len = (TEST_PAYLOAD_LEN - IPV6_OVERHEAD_LEN);
    uint8_t test_char;
    test_char = 'X';

    packet_counter++;
  sprintf(buf, "%08lu", packet_counter);
  memset(&buf[8], test_char, (len - 8));
  memset(&buf[len], '\0', 1);

  printf("sending %d bytes of UDP payload\n", strlen(buf));

  ti_lib_gpio_pin_write(BOARD_IOID_DP0, 1);

  if(packet_counter >= PACKET_OFFSET) {
      latency[packet_counter - PACKET_OFFSET] = RTIMER_NOW();
  }
  uip_udp_packet_send(conn, buf, strlen(buf));
}

void print_latency_data() {
    uint16_t i;
    uint32_t sum = latency[0];
    uint32_t min = latency[0];
    uint32_t max = latency[0];
    for(i = 1; i < PACKET_COUNT; i++) {
        sum += latency[i];
        if(min > latency[i]) {
            min = latency[i];
        }
        if(max < latency[i]) {
            max = latency[i];
        }
    }
    PRINTF("latency of the last %d packets:\n", PACKET_COUNT);
    PRINTF("avg: %lu (ticks)\n", (sum / PACKET_COUNT));
    PRINTF("min: %lu (ticks)\n", min);
    PRINTF("max: %lu (ticks)\n", max);
}
#define BUTTON_GPIO_CFG         (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO | \
                                 IOC_IOPULL_UP    | IOC_SLEW_DISABLE  | \
                                 IOC_HYST_DISABLE | IOC_BOTH_EDGES    | \
                                 IOC_INT_ENABLE   | IOC_IOMODE_NORMAL | \
                                 IOC_NO_WAKE_UP   | IOC_INPUT_ENABLE)
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_client_process, ev, data)
{
  PROCESS_BEGIN();
  PRINTF("BLE UDP client started\n");

  ti_lib_rom_ioc_pin_type_gpio_output(BOARD_IOID_DP0);
  ti_lib_gpio_pin_write((1 << BOARD_IOID_DP0), 1);


  leds_on(LEDS_GREEN);

  /* set address of the server */
  uiplib_ipaddrconv(SERVER_IP, &server_addr);
  /* register echo reply handler */
  uip_icmp6_echo_reply_callback_add(&router_notification, echo_reply_handler);

  /* wait for an echo request/reply from the router to see that the LL connection is established */
  do {
      ti_lib_gpio_pin_toggle((1 << BOARD_IOID_DP0));
    leds_on(LEDS_RED);
    uip_icmp6_send(&server_addr, ICMP6_ECHO_REQUEST, 0, 20);
    etimer_set(&timer, ECHO_TIMEOUT);
    leds_off(LEDS_RED);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
  } while(!echo_received);

  PRINTF("starting client connection\n");

  /* new connection with remote host */
  conn = udp_new(&server_addr, UIP_HTONS(SERVER_PORT), NULL);
  udp_bind(conn, UIP_HTONS(CLIENT_PORT));

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
         UIP_HTONS(conn->lport), UIP_HTONS(conn->rport));

  etimer_set(&timer, SEND_INTERVAL);
  while(1) {
    PROCESS_YIELD();
    if((ev == PROCESS_EVENT_TIMER) && (data == &timer)) {
      timeout_handler();
      etimer_set(&timer, SEND_INTERVAL);

      if(packet_counter > (PACKET_OFFSET +  PACKET_COUNT)) {
          print_latency_data();
          packet_counter=0;
      }

    } else if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
