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
 * ble-controller-cc26xx-ll-ctrl.h
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#ifndef CPU_CC26XX_CC13XX_RF_CORE_BLE_CONTROLLER_BLE_CONTROLLER_CC26XX_LL_CTRL_H_
#define CPU_CC26XX_CC13XX_RF_CORE_BLE_CONTROLLER_BLE_CONTROLLER_CC26XX_LL_CTRL_H_

/*---------------------------------------------------------------------------*/
/* Types of LL control PDUs                                                  */
#define BLE_LL_CONN_UPDATE_REQ              0x00
#define BLE_LL_CHANNEL_MAP_REQ              0x01
#define BLE_LL_TERMINATE_IND                0x02
#define BLE_LL_ENC_REQ                      0x03
#define BLE_LL_ENC_RSP                      0x04
#define BLE_LL_START_ENC_REQ                0x05
#define BLE_LL_START_ENC_RSP                0x06
#define BLE_LL_UNKNOWN_RSP                  0x07
#define BLE_LL_FEATURE_REQ                  0x08
#define BLE_LL_FEATURE_RSP                  0x09
#define BLE_LL_PAUSE_ENC_REQ                0x0A
#define BLE_LL_PAUSE_ENC_RSP                0x0B
#define BLE_LL_VERSION_IND                  0x0C
#define BLE_LL_REJECT_IND                   0x0D
#define BLE_LL_SLAVE_FEATURE_REQ            0x0E
#define BLE_LL_CONN_PARAM_REQ               0x0F
#define BLE_LL_CONN_PARAM_RSP               0x10
#define BLE_LL_REJECT_IND_EXT               0x11
#define BLE_LL_PING_REQ                     0x12
#define BLE_LL_PING_RSP                     0x13
/*---------------------------------------------------------------------------*/
void ble_controller_ll_ctrl_parse_msg(void);

#endif /* CPU_CC26XX_CC13XX_RF_CORE_BLE_CONTROLLER_BLE_CONTROLLER_CC26XX_LL_CTRL_H_ */
