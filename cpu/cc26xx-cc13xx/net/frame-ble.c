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
 * frame-ble.c
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "net/frame-ble.h"
#include "net/packetbuf.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
void print_frame(frame_ble_t *frame){
    int i, length = 0;
    frame_ble_type_t type = frame->frame_type;

    if(type == FRAME_BLE_TYPE_ADV_PDU)
    {
        length = frame->hdr.hdr_adv.length;
        PRINTF("ADV_PDU:\n");
        PRINTF("PDU type:       %d\n", frame->hdr.hdr_adv.pdu_type);
        PRINTF("TxAdd:          %d\n", frame->hdr.hdr_adv.tx_add);
        PRINTF("RxAdd:          %d\n", frame->hdr.hdr_adv.rx_add);
        PRINTF("length:         %d\n", frame->hdr.hdr_adv.length);
    }
    else
    {
        length = frame->hdr.hdr_data.length;
        PRINTF("DATA_PDU:\n");
        PRINTF("LLID:           %d\n", frame->hdr.hdr_data.llid);
        PRINTF("NESN:           %d\n", frame->hdr.hdr_data.nesn);
        PRINTF("SN:             %d\n", frame->hdr.hdr_data.sn);
        PRINTF("MD:             %d\n", frame->hdr.hdr_data.md);
        PRINTF("length:         %d\n", frame->hdr.hdr_data.length);
    }

    if(length > 0)
    {
        PRINTF("payload:        ");
        for(i = 0; i < length; ++i)
        {
            PRINTF("0x%02X ", frame->payload[i]);
        }
        PRINTF("\n");
    }
}

/*---------------------------------------------------------------------------*/
/*
 * Parses incoming data to a ble frame.
 */
int frame_ble_parse(uint8_t *data, int data_length, frame_ble_t *frame)
{
    int channel = packetbuf_attr(PACKETBUF_ATTR_CHANNEL);
    memset(frame, 0x00, sizeof(frame_ble_t));

    if(data_length < 2)
    {
        /* a valid BLE packet (adv and data) has at least a 2-byte header */
        frame->frame_type = FRAME_BLE_TYPE_UNKNOWN;
        frame->payload = NULL;
        return 0;
    }
    if(channel > 36)
    {
        /* frame is advertising pdu */
        frame->frame_type = FRAME_BLE_TYPE_ADV_PDU;
        frame->hdr.hdr_adv.pdu_type = data[0] & 0x0F;
        frame->hdr.hdr_adv.tx_add = (data[0] & 0x40) >> 6;
        frame->hdr.hdr_adv.rx_add = (data[0] & 0x80) >> 7;
        /* the payload length cannot exceed the packetbuf len */
        frame->hdr.hdr_adv.length = MIN(data[1], (data_length - 2));
        frame->payload = &data[2];
    }
    else
    {
        frame->frame_type = FRAME_BLE_TYPE_DATA_PDU;
        frame->hdr.hdr_data.llid = data[0] & 0x03;
        frame->hdr.hdr_data.nesn = (data[0] & 0x04) >> 2;
        frame->hdr.hdr_data.sn = (data[0] & 0x08) >> 3;
        frame->hdr.hdr_data.md = (data[0] & 0x10) >> 4;
        /* the payload length cannot exceed the packetbuf len */
        frame->hdr.hdr_data.length = MIN((data[1] & 0x1F), (data_length - 2));
        frame->payload = &data[2];

        if(frame->hdr.hdr_data.length > 0)
        {
            print_frame(frame);
        }
    }
    return sizeof(frame->hdr);
}
