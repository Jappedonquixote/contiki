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
/*
 * ble-mode.c
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "contiki.h"
#include "dev/radio.h"

#include "dev/ble-controller.h"
#include "rf-core/ble-controller/ble-controller-cc26xx.h"

#include <string.h>
#include <stdio.h>

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
static uint16_t adv_interval;
static ble_adv_type_t adv_type;
static ble_addr_type_t adv_own_addr_type;
static uint8_t adv_channel_map;
/*---------------------------------------------------------------------------*/
static int
init(void)
{
    PRINTF("ble-mode init()\n");
    int result = ble_controller.reset();
    return result == BLE_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static int
prepare(const void *payload, unsigned short payload_len)
{
    PRINTF("ble-mode prepare()\n");
    return 1;
}
/*---------------------------------------------------------------------------*/
static int
transmit(unsigned short transmit_len)
{
    PRINTF("ble-mode transmit()\n");
    return 1;
}
/*---------------------------------------------------------------------------*/
static int
send(const void *payload, unsigned short payload_len)
{
    uint8_t res;
    PRINTF("ble-mode send()\n");
    res = ble_controller.send(payload, payload_len);
    /* always return ok, since the BLE controller handles retransmissions */
    return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
static int
read_frame(void *buf, unsigned short buf_len)
{
    PRINTF("ble-mode read_frame()\n");
    return 0;
}
/*---------------------------------------------------------------------------*/
static int
channel_clear(void)
{
    PRINTF("ble-mode channel_clear()\n");
    return 1;
}
/*---------------------------------------------------------------------------*/
static int
receiving_packet(void)
{
    PRINTF("ble-mode receiving_packet()\n");
    return 0;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
    PRINTF("ble-mode pending_packet()\n");
    return 0;
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
    PRINTF("ble-mode on()\n");
    return 1;
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
    PRINTF("ble-mode off()\n");
    return 1;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
get_value(radio_param_t param, radio_value_t *value)
{
    unsigned int temp;

  if(!value) {
    return RADIO_RESULT_INVALID_VALUE;
  }

  switch(param) {
    case RADIO_CONST_CHANNEL_MIN:
        *value = BLE_DATA_CHANNEL_MIN;
        return RADIO_RESULT_OK;
    case RADIO_CONST_CHANNEL_MAX:
        *value = BLE_DATA_CHANNEL_MAX;
        return RADIO_RESULT_OK;
    case RADIO_CONST_BLE_BUFFER_SIZE:
        ble_controller.read_buffer_size((unsigned int *) value, &temp);
        return RADIO_RESULT_OK;
    case RADIO_CONST_BLE_BUFFER_AMOUNT:
        ble_controller.read_buffer_size(&temp, (unsigned int *) value);
        return RADIO_RESULT_OK;
    case RADIO_CONST_BLE_ADV_INTERVAL_MIN:
        *value = BLE_ADV_INTERVAL_MIN;
        return RADIO_RESULT_OK;
    case RADIO_CONST_BLE_ADV_INTERVAL_MAX:
        *value = BLE_ADV_INTERVAL_MAX;
        return RADIO_RESULT_OK;
    case RADIO_PARAM_BLE_ADV_INTERVAL:
        *value = adv_interval;
        return RADIO_RESULT_OK;
    case RADIO_PARAM_BLE_ADV_TYPE:
        *value = adv_type;
        return RADIO_RESULT_OK;
    case RADIO_PARAM_BLE_ADV_OWN_ADDR_TYPE:
        *value = adv_own_addr_type;
        return RADIO_RESULT_OK;
    case RADIO_PARAM_BLE_ADV_CHANNEL_MAP:
        *value = adv_channel_map;
        return RADIO_RESULT_OK;
    default:
        return RADIO_RESULT_NOT_SUPPORTED;
  }
}
/*---------------------------------------------------------------------------*/
static radio_result_t
set_value(radio_param_t param, radio_value_t value)
{
    switch (param)
    {
    case RADIO_PARAM_BLE_ADV_INTERVAL:
        if((value > BLE_ADV_INTERVAL_MAX) || (value < BLE_ADV_INTERVAL_MIN)) {
            return RADIO_RESULT_INVALID_VALUE;
        }
        adv_interval = (uint16_t) value;
        return RADIO_RESULT_OK;
    case RADIO_PARAM_BLE_ADV_TYPE:
        adv_type = value;
        return RADIO_RESULT_OK;
    case RADIO_PARAM_BLE_ADV_OWN_ADDR_TYPE:
        adv_own_addr_type = value;
        return RADIO_RESULT_OK;
    case RADIO_PARAM_BLE_ADV_CHANNEL_MAP:
        adv_channel_map = value;
        return RADIO_RESULT_OK;
    case RADIO_PARAM_BLE_ADV_ENABLE:
        if(value) {
            /* set the advertisement parameter before enabling */
            ble_controller.set_adv_param(adv_interval, adv_type,
                                         adv_own_addr_type, adv_channel_map);
        }
        ble_controller.set_adv_enable(value);
        return RADIO_RESULT_OK;
        default:
            return RADIO_RESULT_NOT_SUPPORTED;
    }
}
/*---------------------------------------------------------------------------*/
static radio_result_t
get_object(radio_param_t param, void *dest, size_t size)
{
    switch(param) {
        case RADIO_CONST_BLE_BD_ADDR:
            if (size != BLE_ADDR_SIZE || !dest) {
                return RADIO_RESULT_INVALID_VALUE;
            }
            ble_controller.read_bd_addr(dest);
            return RADIO_RESULT_OK;
    }
    return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
set_object(radio_param_t param, const void *src, size_t size)
{
    switch(param) {
        case RADIO_PARAM_BLE_ADV_PAYLOAD:
            if(size <= 0 || size >= BLE_ADV_DATA_LEN || !src) {
                return RADIO_RESULT_INVALID_VALUE;
            }
            ble_controller.set_adv_data((unsigned short) size, (char *) src);
            return RADIO_RESULT_OK;
        case RADIO_PARAM_BLE_ADV_SCAN_RESPONSE:
            if(size <= 0 || size >= BLE_SCAN_RESP_DATA_LEN || !src) {
                return RADIO_RESULT_INVALID_VALUE;
            }
            ble_controller.set_scan_resp_data((unsigned short) size, (char *) src);
            return RADIO_RESULT_OK;
    }
    return RADIO_RESULT_NOT_SUPPORTED;
}

/*---------------------------------------------------------------------------*/
const struct radio_driver ble_mode_driver = {
  init,
  prepare,
  transmit,
  send,
  read_frame,
  channel_clear,
  receiving_packet,
  pending_packet,
  on,
  off,
  get_value,
  set_value,
  get_object,
  set_object,
};
/*---------------------------------------------------------------------------*/
