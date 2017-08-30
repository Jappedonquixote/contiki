/*
 * Copyright (c) 2017, Graz University of Technology
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

/**
 * \file
 *    Adaptive BLE parametrization layer implementation that adapts
 *    the connection interval according to the TX traffic load
 *
 * \author
 *    Michael Spoerk <michael.spoerk@tugraz.at>
 */
/*---------------------------------------------------------------------------*/
#include "ble-hal.h"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include <string.h>
#include "lib/list.h"
#include "sys/ctimer.h"
#include "sys/rtimer.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTADDR(addr) PRINTF(" %02x%02x:%02x%02x:%02x%02x:%02x%02x ", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7])
#else
#define PRINTF(...)
#define PRINTADDR(addr)
#endif
/*---------------------------------------------------------------------------*/
#if !UIP_CONF_ROUTER
#define ADAPTATION_INTERVAL         (4 * CLOCK_SECOND)
#define ADAPTATION_START_TIMEOUT      (30 * CLOCK_SECOND)
#define UTILIZATION_WEIGHT          (0.5f)
#define UTILIZATION_UPPER_THRESHOLD     75
#define UTILIZATION_LOWER_THRESHOLD     25
#define CONN_INTERVAL_MAX          800
#define CONN_INTERVAL_MIN           50

static uint16_t avg_utilization = 0;
static uint16_t tx_requests;
static uint8_t change_pending = 0;
static uint8_t conn_established;
static struct ctimer adaptation_timer;
/*---------------------------------------------------------------------------*/
static void
parameter_adaptation(void *ptr)
{
  uint16_t utilization;
  uint16_t conn_events;
  uint16_t conn_interval;
  uint16_t slave_latency;
  uint16_t conn_interval_new = 0;

  NETSTACK_RADIO.get_value(RADIO_PARAM_BLE_CONN_INTERVAL, (radio_value_t *)&conn_interval);
  NETSTACK_RADIO.get_value(RADIO_PARAM_BLE_CONN_LATENCY, (radio_value_t *)&slave_latency);
  conn_events = (1000 / (conn_interval * 1.25)) * (ADAPTATION_INTERVAL / CLOCK_SECOND);
  utilization = (tx_requests * 100) / conn_events;
  tx_requests = 0;

  if((conn_interval > 0)) {
    if(avg_utilization == 0) {
      avg_utilization = 50;
    } else {
      avg_utilization = UTILIZATION_WEIGHT * utilization + (1 - UTILIZATION_WEIGHT) * avg_utilization;
    }

    if(avg_utilization >= UTILIZATION_UPPER_THRESHOLD) {
      conn_interval_new = conn_interval / 2;
      if(conn_interval_new < CONN_INTERVAL_MIN) {
        conn_interval_new = CONN_INTERVAL_MIN;
      }
    } else if(avg_utilization <= UTILIZATION_LOWER_THRESHOLD) {
      conn_interval_new = conn_interval * 2;
      if(conn_interval_new > CONN_INTERVAL_MAX) {
        conn_interval_new = CONN_INTERVAL_MAX;
      }
    } else {
      conn_interval_new = conn_interval;
    }
    PRINTF("ble-adapt-par: util: %3d, avg_util: %4d, int: %4d, new_int: %4d, change: %d\n",
           utilization, avg_utilization, conn_interval, conn_interval_new, change_pending);
    if((conn_interval != conn_interval_new) && (change_pending == 0)) {
      NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_CONN_INTERVAL, conn_interval_new);
      NETSTACK_RADIO.set_value(RADIO_PARAM_BLE_CONN_UPDATE, 1);
      change_pending = 1;
      PRINTF("ble-adapt-par: changing conn_interval from %4d to %4d (avg. util: %4d)\n",
             conn_interval, conn_interval_new, avg_utilization);
    }
  }

  ctimer_set(&adaptation_timer, ADAPTATION_INTERVAL, parameter_adaptation, NULL);
}
#endif
/*---------------------------------------------------------------------------*/
static void
send_packet(mac_callback_t sent, void *ptr)
{
  int ret;
#if !UIP_CONF_ROUTER
  tx_requests++;
#endif
  if(NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen()) == RADIO_TX_OK) {
    ret = MAC_TX_OK;
  } else {
    ret = MAC_TX_ERR;
  }
  mac_call_sent_callback(sent, ptr, ret, 1);
}
/*---------------------------------------------------------------------------*/
static void
packet_input(void)
{
#if !UIP_CONF_ROUTER
  if(packetbuf_attr(PACKETBUF_ATTR_FRAME_TYPE) == FRAME_BLE_CONNECTION_UPDATED) {
    PRINTF("ble-adapt-par: parameter successfully updated\n");
    change_pending = 0;
  } else if(packetbuf_attr(PACKETBUF_ATTR_FRAME_TYPE) == FRAME_BLE_RX_EVENT) {
    if(conn_established == 0) {
      PRINTF("ble-adapt-par: start_adaptation\n");
      conn_established = 1;
      ctimer_set(&adaptation_timer, ADAPTATION_START_TIMEOUT,
                 parameter_adaptation, NULL);
    }
  }
#endif
  NETSTACK_MAC.input();
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  return NETSTACK_RADIO.on();
}
/*---------------------------------------------------------------------------*/
static int
off(int keep_radio_on)
{
  if(keep_radio_on) {
    return NETSTACK_RADIO.on();
  } else {
    return NETSTACK_RADIO.off();
  }
}
/*---------------------------------------------------------------------------*/
static unsigned short
channel_check_interval(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
init(void)
{
  on();
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver ble_adapt_par_driver = {
  "ble-adapt-par",
  init,
  send_packet,
  NULL,
  packet_input,
  on,
  off,
  channel_check_interval,
};
