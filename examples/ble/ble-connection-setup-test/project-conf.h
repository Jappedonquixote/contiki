/*---------------------------------------------------------------------------*/
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_
/*---------------------------------------------------------------------------*/
/* Disable button shutdown functionality */
#define BUTTON_SENSOR_CONF_ENABLE_SHUTDOWN    0
/*---------------------------------------------------------------------------*/
/* Change to match your configuration */
#define BOARD_CONF_DEBUGGER_DEVPACK			  1

/*---------------------------------------------------------------------------*/
/* network stack settings */
#define NETSTACK_CONF_WITH_IPV6               1
#define NETSTACK_CONF_MAC				ble_mac_driver
#define NETSTACK_CONF_RDC				ble_rdc_driver
#define NETSTACK_CONF_FRAMER            framer_ble
/*---------------------------------------------------------------------------*/
#endif /* PROJECT_CONF_H_ */
/*---------------------------------------------------------------------------*/
