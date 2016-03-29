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
 * ble-rdc.h
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#ifndef BLE_RDC_H_
#define BLE_RDC_H_

#include "contiki.h"

/*---------------------------------------------------------------------------*/
/* advertising parameters */
#define BLE_ADV_INTERVAL_MIN                ( 0.020 * CLOCK_SECOND)
#define BLE_ADV_INTERVAL_MAX                (10.240 * CLOCK_SECOND)
/* use maximum advertising interval as default to preserve battery */
#define BLE_ADV_INTERVAL                    (BLE_ADV_INTERVAL_MIN * 15)

/*---------------------------------------------------------------------------*/
/* connection parameters */
#define BLE_CONN_SUPERVISION_INTERVAL_MIN   ( 0.100 * CLOCK_SECOND)
#define BLE_CONN_SUPERVISION_INTERVAL_MAX   (32.000 * CLOCK_SECOND)
/* use maximum supervision interval as default to preserve battery */
#define BLE_CONN_SUPERVISION_INTERVAL       BLE_CONN_SUPERVISION_INTERVAL_MAX

#define BLE_CONN_INTERVAL_MIN               0x0006      /* 7.5 milliseconds*/
#define BLE_CONN_INTERVAL_MAX               0x0C80      /* 4 seconds */
/* choose the slave connection interval within the defined bounds */
//#define BLE_SLAVE_CONN_INTERVAL_MIN         0x0960      /* 3 seconds */
#define BLE_SLAVE_CONN_INTERVAL_MIN         0x0006
#define BLE_SLAVE_CONN_INTERVAL_MAX         0x0C80      /* 4 seconds */

#define BLE_CONN_SLAVE_LATENCY_MIN          0
#define BLE_CONN_SLAVE_LATENCY_MAX          (BLE_CONN_SUPERVISION_INTERVAL/BLE_CONN_INTERVAL - 1)
/* use maximum slave latency as default to preserve battery */
#define BLE_CONN_SLAVE_LATENCY              BLE_CONN_SLAVE_LATENCY_MAX



#endif /* BLE_RDC_H_ */
