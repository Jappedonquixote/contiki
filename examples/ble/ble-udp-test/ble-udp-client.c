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
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uiplib.h"
#include "net/ipv6/uip-icmp6.h"
#include "dev/leds.h"

#include <string.h>
#include <stdbool.h>

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#define SERVER_IP   "aaaa::1"

#define ECHO_TIMEOUT (CLOCK_SECOND * 5)
#define SEND_INTERVAL       (CLOCK_SECOND * 2)
#define MAX_PAYLOAD_LEN     1280

static struct etimer timer;

//#define ROUTER_ADDR "aaaa::21a:7dff:feda:7114"
#define ROUTER_ADDR "fe80::21a:7dff:feda:7114"

static uip_ipaddr_t router_addr;
static struct uip_icmp6_echo_reply_notification router_notification;
static uint8_t echo_received;

static uip_ipaddr_t server_addr;
static struct uip_udp_conn *conn;

static uint32_t seq_num;
/*---------------------------------------------------------------------------*/
PROCESS(ble_client_process, "BLE UDP client process");
AUTOSTART_PROCESSES(&ble_client_process);
/*---------------------------------------------------------------------------*/
void echo_reply_handler(uip_ipaddr_t *source, uint8_t ttl, uint8_t *data, uint16_t datalen) {
    printf("echo response received\n");
    echo_received = 1;
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *str;

  if(uip_newdata()) {
    str = uip_appdata;
    /* only show the start of the packet */
    str[MIN(uip_datalen(), 40)] = '\0';
    printf("Response from the server: '%s'\n", str);
  }
}
/*---------------------------------------------------------------------------*/
static char buf[MAX_PAYLOAD_LEN];
static void
timeout_handler(void)
{

    // TODO: if the UDP payload is longer than 81 bytes, the whole UDP datagram
    // is not transmitted
    memset(buf, 0x23, (seq_num % 200));
    memset(&buf[seq_num % 200], '\0', 1);

    //  if(seq_num % 10) {
    //      sprintf(buf, "%04lu - this is a test packet", seq_num);
    //  }
    //  else {
    //      sprintf(buf, "%04lu - this packet is a long packet, which does not fit into a single BLE data frame. It is 107 bytes long.", seq_num);
    //  }

//    seq_num++;

    seq_num += 40;

    printf("Send server request     :  %d bytes\n", strlen(buf));
    uip_udp_packet_send(conn, buf, strlen(buf));
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ble_client_process, ev, data)
{
    PROCESS_BEGIN();
    PRINTF("BLE UDP client started\n");

    leds_on(LEDS_GREEN);

    /* set address of router */
    uiplib_ipaddrconv(ROUTER_ADDR, &router_addr);
    uiplib_ipaddrconv(SERVER_IP, &server_addr);
    /* register echo reply handler */
    uip_icmp6_echo_reply_callback_add(&router_notification, echo_reply_handler);

    /* wait for an echo request/reply from the router to see that the LL connection is established */
    do {
        leds_on(LEDS_RED);
        uip_icmp6_send(&server_addr, ICMP6_ECHO_REQUEST, 0, 20);
        printf("echo request sent to server\n");
        etimer_set(&timer, ECHO_TIMEOUT);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    }
    while(!echo_received);

    leds_off(LEDS_RED);
    printf("starting client connection\n");


    /* new connection with remote host */
    conn = udp_new(&server_addr, UIP_HTONS(9000), NULL);
    udp_bind(conn, UIP_HTONS(9001));

    PRINTF("Created a connection with the server ");
    PRINT6ADDR(&conn->ripaddr);
    PRINTF(" local/remote port %u/%u\n",
            UIP_HTONS(conn->lport), UIP_HTONS(conn->rport));

    etimer_set(&timer, SEND_INTERVAL);
    while(1) {
        PROCESS_YIELD();
        if(etimer_expired(&timer)) {
            timeout_handler();
            etimer_restart(&timer);
        } else if(ev == tcpip_event) {
            tcpip_handler();
        }
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
