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
#define TIMEOUT_START       (CLOCK_SECOND * 5)
#define INTERVAL_SEND       (CLOCK_SECOND * 1)
#define MAX_PAYLOAD_LEN     1280

#define IPV6_OVERHEAD_LEN   48
#define TEST_PAYLOAD_LEN    80
static struct etimer timer;

static struct uip_icmp6_echo_reply_notification router_notification;
static uint8_t echo_received;

static uip_ipaddr_t server_addr;
static struct uip_udp_conn *conn;

static uint32_t packet_counter = 0;
static uint32_t sequence_number = 0;
#define PACKET_COUNT         100
/*---------------------------------------------------------------------------*/
PROCESS(ble_client_process, "BLE UDP client process");
AUTOSTART_PROCESSES(&ble_client_process);
/*---------------------------------------------------------------------------*/
void
echo_reply_handler(uip_ipaddr_t *source, uint8_t ttl, uint8_t *data, uint16_t datalen)
{
  PRINTF("echo response received\n");
  echo_received = 1;
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  if(uip_newdata()) {
      if(packet_counter == (PACKET_COUNT - 1)) {
          // stop energy measurement
          ti_lib_gpio_pin_write((1 << BOARD_IOID_DP0), 1);
      }
      PRINTF("UDP packet received\n");
      sequence_number++;
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

    if(packet_counter == 0) {
        // start energy measurement
        ti_lib_gpio_pin_write((1 << BOARD_IOID_DP0), 0);
    }

    sprintf(buf, "%08lu", sequence_number);
    memset(&buf[8], test_char, (len - 8));
    memset(&buf[len], '\0', 1);

    PRINTF("sending UDP packet %lu (sequence_number: %d)\n", packet_counter, sequence_number);
    uip_udp_packet_send(conn, buf, strlen(buf));
    packet_counter++;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_client_process, ev, data)
{
  PROCESS_BEGIN();
  printf("comparison started (%d bytes of IPv6)\n", TEST_PAYLOAD_LEN);

  //////////////////////////////////////////////////////////////////////////////
  // INIT
  ti_lib_rom_ioc_pin_type_gpio_output(BOARD_IOID_DP0);
  ti_lib_gpio_pin_write((1 << BOARD_IOID_DP0), 0);

  /* set address of the server */
  uiplib_ipaddrconv(SERVER_IP, &server_addr);
  /* register echo reply handler */
  uip_icmp6_echo_reply_callback_add(&router_notification, echo_reply_handler);

  //////////////////////////////////////////////////////////////////////////////
  // PING UDP echo server
  /* wait for an echo request/reply from the router to see that the LL connection is established */
  PRINTF("pinging the UDP echo server\n");
  do {
    uip_icmp6_send(&server_addr, ICMP6_ECHO_REQUEST, 0, 20);
    etimer_set(&timer, ECHO_TIMEOUT);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
  } while(!echo_received);
  ti_lib_gpio_pin_write((1 << BOARD_IOID_DP0), 1);

  /* new connection with remote host */
  conn = udp_new(&server_addr, UIP_HTONS(SERVER_PORT), NULL);
  udp_bind(conn, UIP_HTONS(CLIENT_PORT));

  //////////////////////////////////////////////////////////////////////////////
  // wait for the connection to be more stable
  PRINTF("waiting for stable connection \n");
  etimer_set(&timer, TIMEOUT_START);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

  etimer_set(&timer, INTERVAL_SEND);
  while(1) {
    PROCESS_YIELD();
    if((ev == PROCESS_EVENT_TIMER) && (data == &timer)) {
      timeout_handler();
      etimer_set(&timer, INTERVAL_SEND);
    } else if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
