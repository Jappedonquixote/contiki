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
/*
 * ble-mac.c
 *
 *      Author: Michael Spörk
 */

#include "net/ble-mac.h"
#include "net/ble-rdc.h"
#include "net/packetbuf.h"
#include "net/netstack.h"
#include "net/frame-ble.h"

#include "net/ble-l2cap.h"
#include "rf-core/ble-stack/ble-radio-controller.h"

#include <string.h>

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
static void init(void)
{
    PRINTF("[ ble-mac ] init()\n");
}

/*---------------------------------------------------------------------------*/
static void send(mac_callback_t sent_callback, void *ptr)
{
//    PRINTF("[ ble-mac ] send()\n");
    NETSTACK_RDC.send(sent_callback, ptr);
}

/*---------------------------------------------------------------------------*/
void process_l2cap_frame(ble_l2cap_frame_t *frame)
{
    uint8_t cmd_id;
    ble_l2cap_frame_t rsp_frame;



    uint8_t data[25];
    uint8_t len;



    if(frame->type == BLE_L2CAP_C_FRAME)
    {
        /* C-frame received */

        // TODO: implement L2CAP connection functionality and push data onto packetbuf
        /* set LLID control frame */
        data[0] = 0x02;

        data[1] = 0x0E;
        data[2] = 0x00;
        data[3] = 0x05;
        data[4] = 0x00;
        data[5] = BLE_L2CAP_CONN_RSP_CODE;
        data[6] = frame->cmd.conn_req.cmd_id;
        data[7] = 0x0A;
        data[8] = 0x00;
        data[9] = 0x69;
        data[10] = 0x00;
        data[11] = 0xFF;
        data[12] = 0xFF;
        data[13] = 0xE0;
        data[14] = 0x00;
        data[15] = 0x0A;
        data[16] = 0x00;
        data[17] = 0x00;
        data[18] = 0x00;

        len = 19;

        ble_radio_controller_send(data, len);
    }
}

/*---------------------------------------------------------------------------*/
static void input(void)
{
    uint8_t len = packetbuf_datalen();
    ble_l2cap_frame_t frame;

    if(len > 0)
    {
        ble_l2cap_parse((uint8_t *) packetbuf_dataptr(), len, &frame);
        process_l2cap_frame(&frame);
    }
}

/*---------------------------------------------------------------------------*/
static int on(void)
{
    PRINTF("[ ble-mac ] on()\n");
    return NETSTACK_RDC.on();
}

/*---------------------------------------------------------------------------*/
static int off(int keep_radio_on)
{
    PRINTF("[ ble-mac ] off()\n");
    return NETSTACK_RDC.off(keep_radio_on);
}

/*---------------------------------------------------------------------------*/
static unsigned short channel_check_interval(void)
{
    PRINTF("[ ble-mac ] channel_check_interval()\n");
    return 0;
}

/*---------------------------------------------------------------------------*/
const struct mac_driver ble_mac_driver = {
  "ble_mac",
  init,
  send,
  input,
  on,
  off,
  channel_check_interval,
};
