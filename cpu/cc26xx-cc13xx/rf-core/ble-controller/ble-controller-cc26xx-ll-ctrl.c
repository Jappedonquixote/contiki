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
 * ble-controller-cc26xx-ll-ctrl.c
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "ble-controller-cc26xx-ll-ctrl.h"
#include "ble-controller-cc26xx.h"

#include "net/packetbuf.h"
#include "net/frame-ble.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* controller specific LL fields                                             */
#define BLE_VERSION_4_0                     6
#define BLE_VERSION_4_1                     7
#define BLE_VERSION_4_2                     8
#define BLE_VERSION_NR        BLE_VERSION_4_1

#define BLE_COMPANY_ID                 0xFFFF
#define BLE_SUB_VERSION_NR             0xBEEF

/*---------------------------------------------------------------------------*/
void ble_controller_ll_ctrl_parse_msg(void)
{
    uint8_t resp_len = 0;
    uint8_t resp_data[26];
    uint8_t *data = packetbuf_dataptr();
    uint8_t op_code = data[0];

    if(op_code == BLE_LL_FEATURE_REQ) {
        resp_data[0] = BLE_LL_FEATURE_RSP;
        memset(&resp_data[1], 0x00, 8);
        resp_len = 9;
    }
    else if(op_code == BLE_LL_VERSION_IND) {
        resp_data[0] = BLE_LL_VERSION_IND;
        resp_data[1] = BLE_VERSION_NR;
        resp_data[2] = (BLE_COMPANY_ID >> 8) & 0xFF;
        resp_data[3] = BLE_COMPANY_ID & 0xFF;
        resp_data[4] = (BLE_SUB_VERSION_NR >> 8) & 0xFF;
        resp_data[5] = BLE_SUB_VERSION_NR & 0xFF;
        resp_len = 6;
    }
    else {
        PRINTF("parse_ll_ctrl_msg() opcode: 0x%02X received\n", op_code);
    }

    if(resp_len > 0) {
        /* write response into packet buffer */
        packetbuf_copyfrom(resp_data, resp_len);
        packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME_BLE_TYPE_DATA_LL_CTRL);
        ble_controller.send();
    }
}
