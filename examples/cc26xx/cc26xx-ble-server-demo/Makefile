DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT = server

CONTIKI_CPU_SOURCEFILES += ble-cc2650-ext.c ble-hal-cc26xx-ext.c

all: $(CONTIKI_PROJECT)

CONTIKI_WITH_RPL = 0
CONTIKI_WITH_IPV6 = 1

CONTIKI = ../../..
include $(CONTIKI)/Makefile.include