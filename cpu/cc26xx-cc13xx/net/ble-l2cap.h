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
 * ble-l2cap.h
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#ifndef BLE_L2CAP_H_
#define BLE_L2CAP_H_

#include "contiki-conf.h"

#define BLE_L2CAP_CMD_OK     1
#define BLE_L2CAP_CMD_ERROR  0

#define BLE_L2CAP_SIGNAL_CHAN_ID        0x0005

typedef enum {
    BLE_L2CAP_C_FRAME
} ble_l2cap_frame_type_t;

#define BLE_L2CAP_CONN_REQ_CODE 0x14
typedef struct {
    uint8_t code;
    uint8_t cmd_id;
    uint16_t cmd_len;
    uint16_t le_psm;
    uint16_t source_cid;
    uint16_t mtu;
    uint16_t mps;
    uint16_t initial_credits;
} ble_l2cap_conn_req_t;

#define BLE_L2CAP_CONN_RSP_CODE 0x15
typedef struct {
    uint8_t code;
    uint8_t cmd_id;
    uint16_t cmd_len;
    uint16_t dest_cid;
    uint16_t mtu;
    uint16_t mps;
    uint16_t initial_credits;
    uint16_t result;
} ble_l2cap_conn_rsp_t;

typedef union {
    ble_l2cap_conn_req_t conn_req;
    ble_l2cap_conn_rsp_t conn_rsp;
} ble_l2cap_cmd_t;

typedef struct {
    ble_l2cap_frame_type_t type;
    uint16_t frame_len;
    uint16_t channel_id;
    ble_l2cap_cmd_t cmd;
} ble_l2cap_frame_t;


unsigned int ble_l2cap_parse(uint8_t *data, uint8_t data_len, ble_l2cap_frame_t *frame);

#endif /* BLE_L2CAP_H_ */
