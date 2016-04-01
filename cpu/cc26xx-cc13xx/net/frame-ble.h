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
 * frame-ble.h
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#ifndef FRAME_BLE_H_
#define FRAME_BLE_H_

#include "contiki-conf.h"

#include "rf-core/ble-stack/ble-addr.h"

#define FRAME_BLE_ADV_PDU_ADV_IND               0b0000
#define FRAME_BLE_ADV_PDU_ADV_DIRECTED_IND      0b0001
#define FRAME_BLE_ADV_PDU_ADV_NONCONN_IND       0b0010
#define FRAME_BLE_ADV_PDU_SCAN_REQ              0b0011
#define FRAME_BLE_ADV_PDU_SCAN_RSP              0b0100
#define FRAME_BLE_ADV_PDU_CONNECT_REQ           0b0101
#define FRAME_BLE_ADV_PDU_ADV_SCAN_IND          0b0110

/*
 * Frame type of the current BLE frame.
 */
typedef enum {
    FRAME_BLE_TYPE_ADV_PDU,
    FRAME_BLE_TYPE_DATA_PDU
} frame_ble_type_t;

/*
 * header of BLE advertising PDU
 */
typedef struct {
    uint8_t pdu_type:4;     /* PDU type field (e.g. scan request, ...), see BLE specification */
//    uint8_t reserved:2;     /* reserved bits */
    uint8_t tx_add:1;       /* TxAdd, see BLE specification */
    uint8_t rx_add:1;       /* RxAdd, see BLE specification */
    uint8_t length;         /* advertising payload length */
} frame_ble_adv_hdr_t;

/*
 * header of BLE data PDU
 */
typedef struct {
    uint8_t llid:2;         /* packet indicator */
    uint8_t nesn:1;         /* next expected sequence number */
    uint8_t sn:1;           /* sequence number */
    uint8_t md:1;           /* more data */
//    uint8_t reserved:3;   /* reserved */
    uint8_t length:5;       /* data payload length */
//    uint8_t reserved:3;   /* reserved */
} frame_ble_data_hdr_t;

/*
 * BLE header
 */
typedef union {
    frame_ble_adv_hdr_t hdr_adv;
    frame_ble_data_hdr_t hdr_data;
} frame_ble_hdr_t;

typedef struct {
    frame_ble_type_t frame_type;
    frame_ble_hdr_t hdr;        /* PDU header */
    uint8_t *payload;           /* payload */
} frame_ble_t;

int frame_ble_parse(uint8_t *data, int data_length, frame_ble_t *frame);

#endif /* FRAME_BLE_H_ */
