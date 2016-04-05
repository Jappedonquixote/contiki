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
 * ble-l2cap.c
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "net/ble-l2cap.h"

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
//void print_frame(frame_ble_t *frame){
//    int i, length = 0;
//    frame_ble_type_t type = frame->frame_type;
//
//    if(type == FRAME_BLE_TYPE_ADV_PDU)
//    {
//        length = frame->hdr.hdr_adv.length;
//        PRINTF("ADV_PDU:\n");
//        PRINTF("PDU type:       %d\n", frame->hdr.hdr_adv.pdu_type);
//        PRINTF("TxAdd:          %d\n", frame->hdr.hdr_adv.tx_add);
//        PRINTF("RxAdd:          %d\n", frame->hdr.hdr_adv.rx_add);
//        PRINTF("length:         %d\n", frame->hdr.hdr_adv.length);
//    }
//    else
//    {
//        length = frame->hdr.hdr_data.length;
//        PRINTF("DATA_PDU:\n");
//        PRINTF("LLID:           %d\n", frame->hdr.hdr_data.llid);
//        PRINTF("NESN:           %d\n", frame->hdr.hdr_data.nesn);
//        PRINTF("SN:             %d\n", frame->hdr.hdr_data.sn);
//        PRINTF("MD:             %d\n", frame->hdr.hdr_data.md);
//        PRINTF("length:         %d\n", frame->hdr.hdr_data.length);
//    }
//
//    if(length > 0)
//    {
//        PRINTF("payload:        ");
//        for(i = 0; i < length; ++i)
//        {
//            PRINTF("0x%02X ", frame->payload[i]);
//        }
//        PRINTF("\n");
//    }
//}


void print_l2cap_frame(ble_l2cap_frame_t *frame)
{
    ble_l2cap_frame_type_t type = frame->type;

    PRINTF("L2CAP FRAME: length: %d, channel ID: 0x%04X\n",
           frame->frame_len, frame->channel_id);

    if(type == BLE_L2CAP_C_FRAME)
    {
        PRINTF("  L2CAP C-FRAME: code: 0x%02X, cmd_id: 0x%02X, cmd_len: 0x%04X\n",
               frame->cmd.conn_req.code, frame->cmd.conn_req.cmd_id,
               frame->cmd.conn_req.cmd_len);
        if(frame->cmd.conn_req.code == BLE_L2CAP_CONN_REQ_CODE)
        {
            PRINTF("    LE Credit based CONN REQ:\n");
            PRINTF("    le_psm:          0x%04X (%8d)\n",
                   frame->cmd.conn_req.le_psm, frame->cmd.conn_req.le_psm);
            PRINTF("    source_CID:      0x%04X (%8d)\n",
                   frame->cmd.conn_req.source_cid, frame->cmd.conn_req.source_cid);
            PRINTF("    MTU:             0x%04X (%8d)\n",
                   frame->cmd.conn_req.mtu, frame->cmd.conn_req.mtu);
            PRINTF("    MPS:             0x%04X (%8d)\n",
                   frame->cmd.conn_req.mps, frame->cmd.conn_req.mps);
            PRINTF("    initial_credits: 0x%04X (%8d)\n",
                   frame->cmd.conn_req.initial_credits, frame->cmd.conn_req.initial_credits);
        }
        else
        {
            PRINTF("    unknown command code\n");
        }
    }
    else
    {
        PRINTF("  unknown L2CAP frame\n");
    }
}

/*---------------------------------------------------------------------------*/
unsigned int ble_l2cap_parse(uint8_t *data, uint8_t data_len, ble_l2cap_frame_t *frame)
{
    memset(frame, 0x00, sizeof(ble_l2cap_frame_t));

    if(data_len < 2)
    {
        /* a valid L2CAP packet needs to be at least 2 bytes long */
        return BLE_L2CAP_CMD_ERROR;
    }

    frame->frame_len = (data[1] << 8) + data[0];
    frame->channel_id = (data[3] << 8) + data[2];

    if(frame->channel_id == BLE_L2CAP_SIGNAL_CHAN_ID)
    {
        frame->type = BLE_L2CAP_C_FRAME;

        if(data[4] != BLE_L2CAP_CONN_REQ_CODE)
        {
            PRINTF("ble_l2cap_parse: unknown code (0x%02X) on signal channel\n",
                   data[4]);
        }

        frame->cmd.conn_req.code = data[4];
        frame->cmd.conn_req.cmd_id = data[5];
        frame->cmd.conn_req.cmd_len = (data[7] << 8) + data[6];
        frame->cmd.conn_req.le_psm = (data[9] << 8) + data[8];
        frame->cmd.conn_req.source_cid = (data[11] << 8) + data[10];
        frame->cmd.conn_req.mtu = (data[13] << 8) + data[12];
        frame->cmd.conn_req.mps = (data[15] << 8) + data[14];
        frame->cmd.conn_req.initial_credits = (data[17] << 8) + data[16];

        print_l2cap_frame(frame);

        return BLE_L2CAP_CMD_OK;
    }
    else
    {
        PRINTF("ble_l2cap_parse: unknown channel id: %d\n", frame->channel_id);
        return BLE_L2CAP_CMD_ERROR;
    }
}
