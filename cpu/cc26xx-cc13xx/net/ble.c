/*
 * ble.c
 *
 *  Created on: 07. Dez. 2015
 *      Author: Michi
 */

#include "ble.h"
#include "net/netstack.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

int ble_nc_advertisement(void)
{
	PRINTF("ble_nc_advertisement called\n");
//	NETSTACK_RADIO.send(0, 0);
	return 0;
}
