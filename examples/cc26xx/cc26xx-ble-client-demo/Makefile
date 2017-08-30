DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT=client

CONTIKI_CPU_SOURCEFILES += ble-adapt-par.c

all: $(CONTIKI_PROJECT)

CONTIKI_WITH_RPL = 0
CONTIKI_WITH_IPV6 = 1

CONTIKI = ../../..
include $(CONTIKI)/Makefile.include