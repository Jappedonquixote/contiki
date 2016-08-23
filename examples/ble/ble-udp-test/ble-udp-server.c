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
 *       A test for the Bluetooth Low-Energy radio of Contiki
 * \author
 *         Michael Spörk <m.spoerk@student.tugraz.at>
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ipv6/uip-icmp6.h"
#include "dev/leds.h"
#include "sys/etimer.h"

#include <stdio.h>
#include <string.h>

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define MAX_PAYLOAD_LEN 50
#define ECHO_TIMEOUT (CLOCK_SECOND * 5)

#define SEND_INTERVAL       10 * CLOCK_SECOND
static struct etimer timer;

/*#define ROUTER_ADDR "aaaa::21a:7dff:feda:7114" */
#define ROUTER_ADDR "fe80::21a:7dff:feda:7114"

static uip_ipaddr_t router_addr;
static struct uip_icmp6_echo_reply_notification router_notification;
static uint8_t echo_received;

#define SERVER_IP   "aaaa::6ac9:bff:fe07:170d"

static uip_ipaddr_t client_addr;
static struct uip_udp_conn *conn;
/*---------------------------------------------------------------------------*/
PROCESS(ble_server_process, "BLE UDP server process");
AUTOSTART_PROCESSES(&ble_server_process);
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
  static int seq_id;
  char buf[MAX_PAYLOAD_LEN];

  if(uip_newdata()) {
    ((char *)uip_appdata)[uip_datalen()] = 0;
    PRINTF("Server received: '%s' from ", (char *)uip_appdata);
    PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
    PRINTF("\n");

    uip_ipaddr_copy(&conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    PRINTF("Responding with message: ");
    sprintf(buf, "Hello from the server! (%d)", ++seq_id);
    PRINTF("%s\n", buf);

    uip_udp_packet_send(conn, buf, strlen(buf));
    /* Restore server connection to allow data from any node */
    memset(&conn->ripaddr, 0, sizeof(conn->ripaddr));
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_server_process, ev, data)
{
  PROCESS_BEGIN();
  leds_on(LEDS_GREEN);

  printf("BLE UDP server started\n");

  /* set address of router */
  uiplib_ipaddrconv(ROUTER_ADDR, &router_addr);
  /* register echo reply handler */
  uip_icmp6_echo_reply_callback_add(&router_notification, echo_reply_handler);

  /* wait for an echo request/reply from the router to see that the LL connection is established */
  do {
    leds_on(LEDS_RED);
    uip_icmp6_send(&router_addr, ICMP6_ECHO_REQUEST, 0, 20);
    printf("echo request sent to router\n");
    etimer_set(&timer, ECHO_TIMEOUT);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
  } while(!echo_received);

  leds_off(LEDS_RED);
  uiplib_ipaddrconv(SERVER_IP, &client_addr);

  conn = udp_new(NULL, UIP_HTONS(3001), NULL);
  udp_bind(conn, UIP_HTONS(3000));

  PRINTF("Created server socket local/remote port %u/%u\n",
         UIP_HTONS(conn->lport), UIP_HTONS(conn->rport));

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
