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
 * ble-controller.h
 *
 *      Author: Michael Spoerk <m.spoerk@student.tugraz.at>
 */

#ifndef CPU_CC26XX_CC13XX_RF_CORE_BLE_STACK_BLE_RADIO_ADVERTISING_H_
#define CPU_CC26XX_CC13XX_RF_CORE_BLE_STACK_BLE_RADIO_ADVERTISING_H_

#define BLE_ADV_CHANNEL_1_MASK          0b0001
#define BLE_ADV_CHANNEL_2_MASK          0b0010
#define BLE_ADV_CHANNEL_3_MASK          0b0100
#define BLE_ADV_CHANNEL_MASK_MIN        0b0001
#define BLE_ADV_CHANNEL_MASK_MAX        0b0111
#define BLE_ADV_CHANNEL_1               37
#define BLE_ADV_CHANNEL_2               38
#define BLE_ADV_CHANNEL_3               39

#define BLE_ADV_DATA_LENGHT_MAX         31
#define BLE_SCAN_RESP_DATA_LENGHT_MAX   31

#define BLE_COMMAND_BUFFER_LENGTH       24
#define BLE_PARAMS_BUFFER_LENGTH        32
#define BLE_RECEIVE_BUFFER_LENGTH       56
#define BLE_OUTPUT_BUFFER_LENGTH        32

/*---------------------------------------------------------------------------*/
/* advertising parameters */
#define BLE_ADV_INTERVAL_MIN                 0.020
#define BLE_ADV_INTERVAL_MAX                10.240


#define BLE_ADV_INTERVAL                    1       /* send advertisement each 1 second*/
#define BLE_ADV_CHANNEL_MASK    BLE_ADV_CHANNEL_1_MASK
#define BLE_DEVICE_NAME         "msp - TI SensorTag"

#define BLE_CONN_INTERVAL_MIN               0x0006      /* 7.5 milliseconds*/
#define BLE_CONN_INTERVAL_MAX               0x0C80      /* 4 seconds */


#define BLE_SLAVE_CONN_INTERVAL_MIN         0x0960      /* 3 seconds */
#define BLE_SLAVE_CONN_INTERVAL_MAX         0x0C80      /* 4 seconds */

/*---------------------------------------------------------------------------*/
extern process_event_t conn_req_event;
/*---------------------------------------------------------------------------*/
/* This struct contains all data provided from the master with a conn request*/
typedef struct {
    uint32_t access_address;
    uint8_t crc_init_0;
    uint8_t crc_init_1;
    uint8_t crc_init_2;
    uint8_t win_size;
    uint16_t win_offset;
    uint16_t interval;
    uint16_t latency;
    uint16_t timeout;
    uint64_t channel_map;
    uint8_t hop;
    uint8_t sca;
    uint32_t timestamp;
} ble_conn_req_data_t;

/*---------------------------------------------------------------------------*/
//unsigned short ble_radio_advertising_set_advertising_parameters(
//        unsigned int advertising_interval, char advertising_channel_map);
//
//unsigned short ble_radio_advertising_set_advertising_data(
//        unsigned int advertising_data_length, char* advertising_data);
//
//unsigned short ble_radio_advertising_set_scan_response_data(
//        unsigned int scan_response_data_length, char* scan_response_data);

unsigned short ble_radio_advertising_enable_advertising(void);

unsigned short ble_radio_advertising_disable_advertising(void);

#endif /* CPU_CC26XX_CC13XX_RF_CORE_BLE_STACK_BLE_RADIO_ADVERTISING_H_ */
