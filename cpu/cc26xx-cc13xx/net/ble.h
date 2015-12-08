/*
 * ble.h
 *
 *  Created on: 07. Dez. 2015
 *      Author: Michael Spoerk
 */

#ifndef CPU_CC26XX_CC13XX_NET_BLE_H_
#define CPU_CC26XX_CC13XX_NET_BLE_H_

/* Used for return values for ble commands */
#define BLE_CMD_OK 		1
#define BLE_CMD_ERROR 	0

/**
 * \brief Sends a single advertisement message.
 * \return BLE_CMD_OK or BLE_CMD_ERROR
 */
int ble_send_advertisement(int channel, char *payload, int payload_len);

#endif /* CPU_CC26XX_CC13XX_NET_BLE_H_ */
