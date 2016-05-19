DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT = ble-udp-test
all: ble-udp-server ble-udp-client

CONTIKI_WITH_RPL = 0
CONTIKI_WITH_IPV6 = 1

CONTIKI = ../../..
include $(CONTIKI)/Makefile.include

upload-client:
	$(SRFPROG) -t 'soc(XDS-0000000C)' -e all -p 'epfw(0)' -v rb -f ble-udp-client.hex -a 0x0
	
upload-server:
	$(SRFPROG) -t 'soc(XDS-0000000S)' -e all -p 'epfw(0)' -v rb -f ble-udp-server.hex -a 0x0

upload-both:
	$(SRFPROG) -t 'soc(XDS-0000000C)' -e all -p 'epfw(0)' -v rb -f ble-udp-client.hex -a 0x0
	$(SRFPROG) -t 'soc(XDS-0000000S)' -e all -p 'epfw(0)' -v rb -f ble-udp-server.hex -a 0x0