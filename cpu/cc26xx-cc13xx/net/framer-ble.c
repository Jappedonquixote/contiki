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
 * framer-ble.c
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#include "net/framer-ble.h"
#include "net/frame-ble.h"

#include "net/packetbuf.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINTADDR(addr) PRINTF(" %02X:%02X:%02X:%02X:%02X:%02X ", ((uint8_t *)addr)[5], ((uint8_t *)addr)[4], ((uint8_t *)addr)[3], ((uint8_t *)addr)[2], ((uint8_t *)addr)[1], ((uint8_t *)addr)[0])

#else
#define PRINTF(...)
#define PRINTADDR(addr)
#endif


/*---------------------------------------------------------------------------*/
int length(void)
{

}

/*---------------------------------------------------------------------------*/
int create(void)
{
    return FRAMER_FAILED;
}


/*---------------------------------------------------------------------------*/
int parse(void)
{
    int hdr_len;
    int i;
    char *buf = packetbuf_dataptr();
    frame_ble_t frame;
    hdr_len = frame_ble_parse(packetbuf_dataptr(), packetbuf_datalen(), &frame);

    if(hdr_len && packetbuf_hdrreduce(hdr_len)) {
        if(frame.frame_type == FRAME_BLE_TYPE_ADV_PDU) {
            packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, frame.hdr.hdr_adv.pdu_type);
        }
        return hdr_len;
    }
    return FRAMER_FAILED;
}

/*---------------------------------------------------------------------------*/
const struct framer framer_ble = {
  length,
  create,
  parse
};
