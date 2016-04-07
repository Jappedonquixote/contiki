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
 * ble-rdc.c
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "net/ble-rdc.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"

#include "net/framer-ble.h"
#include "net/frame-ble.h"

#include "rf-core/ble-stack/ble-radio-controller.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif



/*---------------------------------------------------------------------------*/
static void send(mac_callback_t sent_callback, void *ptr)
{
//    PRINTF("[ ble-rdc ] send()\n");
    if(NETSTACK_FRAMER.create() < 0)
    {
        PRINTF("ble-rdc: could not create frame\n");
        return;
    }

    ble_radio_controller_send(packetbuf_hdrptr(), packetbuf_totlen());
}

/*---------------------------------------------------------------------------*/
static void send_list(mac_callback_t sent_callback, void *ptr, struct rdc_buf_list *list)
{
    PRINTF("[ ble-rdc ] send_list()\n");
    send(sent_callback, ptr);
}

/*---------------------------------------------------------------------------*/
void process_llid_control_mesg(uint8_t *payload)
{
    uint8_t opcode = payload[0];
    uint8_t response_data[26];
    uint8_t response_len = 0;

    if(opcode == FRAME_BLE_LL_FEATURE_REQ)
    {
        PRINTF("[ ble-rdc ] LL_FEATURE_REQ received\n");
        /* set LLID control frame */
        response_data[0] = 0x03;

        /* set LLID opcode*/
        response_data[1] = FRAME_BLE_LL_FEATURE_RSP;

        /* set feature_set to 0 (no feature supported) */
        memset(&response_data[2], 0x00, 8);

        response_len = 10;
        ble_radio_controller_send(response_data, response_len);
    }
    else if(opcode == FRAME_BLE_LL_VERSION_IND)
    {
        PRINTF("[ ble-rdc ] LL_VERSION_IND received\n");
        /* set LLID control frame */
        response_data[0] = 0x03;

        /* set LLID opcode*/
        response_data[1] = FRAME_BLE_LL_VERSION_IND;

        // TODO: send the real version and company ID
        response_data[2] = 0x06;
        response_data[3] = 0x00;
        response_data[4] = 0x0A;
        response_data[5] = 0xFE;
        response_data[6] = 0xCA;

        response_len = 7;
        ble_radio_controller_send(response_data, response_len);
    }
    else
    {
        PRINTF("[ ble-rdc ] input() control frame (opcode: 0x%0X) received\n",
                opcode);
    }
}
/*---------------------------------------------------------------------------*/
static void input(void)
{
    uint8_t hdr_len;
    frame_ble_t frame;


    hdr_len = framer_ble_parse_frame(&frame);
    if(hdr_len < 0)
    {
        PRINTF("[ ble-rdc ] input() could not parse frame\n");
        return;
    }

    if((frame.frame_type == FRAME_BLE_TYPE_DATA_PDU) &&
       (frame.hdr.hdr_data.llid == FRAME_BLE_DATA_PDU_LLID_CONTROL))
    {
        /* received frame is a LL control frame */
        process_llid_control_mesg(frame.payload);
    }
    else
    {
        /* LLID messages and fragments are handled in the mac layer */
        NETSTACK_MAC.input();
    }
}

/*---------------------------------------------------------------------------*/
static int on(void)
{
    PRINTF("[ ble-rdc ] on()\n");
    return 1;
}

/*---------------------------------------------------------------------------*/
static int off(int keep_radio_on)
{
    PRINTF("[ ble-rdc ] off()\n");
    if(keep_radio_on) {
    		return NETSTACK_RADIO.on();
    }
    else {
            return NETSTACK_RADIO.off();
    }
}

/*---------------------------------------------------------------------------*/
static unsigned short channel_check_interval(void)
{
    PRINTF("[ ble-rdc ] channel_check_interval()\n");
    return 0;
}

/*---------------------------------------------------------------------------*/
static void init(void)
{
    PRINTF("[ ble-rdc ] init()\n");
    ble_radio_controller_init();
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver ble_rdc_driver = {
  "ble-rdc",
  init,
  send,
  send_list,
  input,
  on,
  off,
  channel_check_interval,
};
