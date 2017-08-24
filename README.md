# BLEach: Exploiting the Full Potential of IPv6 over BLE in Constrained Embedded IoT Devices

BLEach is a fully open-source IPv6-over-BLE stack for the Contiki OS.
The stack implements full support for IPv6-over-BLE node (BLE slave) and border router (BLE master) devices according to the [RFC 7668][rfc7668]. 
BLEach is interoperable with other RFC-7668-compliant devices and is able to exchange IPv6 packets with a maximum packet length of 1280 bytes.

BLEach is presented and evaluated in detail in the paper:<br/>
Michael Spörk, Carlo Alberto Boano, Marco Zimmerling, and Kay Römer. 
**BLEACH: Exploiting the Full Potential of IPv6 over BLE in Constrained Embedded IoT Devices**, 
_In Proceedings of the 15th ACM Conference on Embedded Networked Sensor Systems (SenSys), Delft (The Netherlands), November 2017_ <br/>
\[[Link](http://sensys.acm.org/2017/)\] \[[Paper](http://sensys.acm.org/2017/)\]

## Features
BLEach provides full support both IPv6-over-BLE nodes and border routers compliant to [RFC 7668][rfc7668].
Currently, BLEach only supports the [Texas Instruments CC2650](http://www.ti.com/tool/cc2650stk)  hardware platform.

_IPv6-over-BLE node (BLE slave)_:
 * connect to a single IPv6-over-BLE border router
 * maximum IPv6 packet length of 1280 bytes
 * BLE L2CAP channels in LE credit-based flow control mode
 
_IPv6-over-BLE routers (BLE master)_:
 * connect to up to 4 different IPv6-over-BLE border nodes
 * maximum IPv6 packet length of 1280 bytes
 * BLE L2CAP channels in LE credit-based flow control mode
 
BLEach provides the following BLE link layer functionality according to the [Bluetooth Specification v4.1][bleSpec]:
 * BLE advertisement
 * BLE initiating
 * BLE connection slave
 * BLE connection master
  	
It has been tested on the TI CC2650 SensorTag and the TI CC2650 LaunchPad hardware.

## Architecture

BLEach is fully compatible with the architecture of the Contiki network stack.
It reuses the IPv6 and UDP support and maps each of its four lowest stack layers to an existing layer in Contiki's stack.

![BLEach architecture](img/bleach-architecture.png)

### BLE radio
BLEach's radio layer, the BLE link and physical layer, directly maps into Contiki's radio layer, but provides completely different functionality. This layer decouples the radio from the upper stack layers, _prohibiting direct radio access_.
These files contain all the hardware specific code for supporting BLE.

The implementation of the BLE radio for the TI CC26xx platform is implemented in `cpu/cc26xx-cc13xx/rf-core/ble-cc2650.c`
and `cpu/cc26xx-cc13xx/rf-core/ble-hal/*.[ch]`.

### BLE parametrization layer
The parametrization layer maps into Contiki's radio duty cycling (RDC) layer.
Since directly turning the radio on and off is not possible, this layer _indirectly_ changes the radio duty cycle by changing the BLE connection parameters.
A minimal version of the BLE parametrization layer is implemented in `cpu/cc26xx-cc13xx/net/ble-null-par.[ch]`.
In this minimal version the parametrization does not perform any parameter adaptation.

### BLE L2CAP layer
BLEach'S L2CAP layer directly maps into Contiki's medium access control (MAC) layer.
This layer provides support for fragmentation of large IPv6 packets and flow control.
The L2CAP LE credit-based flow control support is implemented in `cpu/cc26xx-cc13xx/net/ble-l2cap.c`.

## Using BLEach
The [README](https://github.com/spoerk/contiki/blob/master/cpu/cc26xx-cc13xx/net/README.md) in the [git-repository](https://github.com/spoerk/contiki) provides information on how to use BLEach.
In addition, a simple IPv6-over-BLE client and server example is included under [`examples/cc26xx/cc26xx-ble-client-demo`](https://github.com/spoerk/contiki/tree/master/examples/cc26xx/cc26xx-ble-client-demo)
and [`examples/cc26xx/cc26xx-ble-server-demo`](https://github.com/spoerk/contiki/tree/master/examples/cc26xx/cc26xx-ble-server-demo), respectively.


[rfc7668]: https://tools.ietf.org/html/rfc7668
[bleSpec]: https://www.bluetooth.com/specifications/bluetooth-core-specification/legacy-specifications
[bleachWeb]: http://www.iti.tugraz.at/BLEach

