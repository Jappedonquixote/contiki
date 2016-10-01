DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT = comparison
all: $(CONTIKI_PROJECT)

CONTIKI_WITH_RPL = 0
CONTIKI_WITH_IPV6 = 1

CONTIKI = ../../..
include $(CONTIKI)/Makefile.include

#upload:
#	$(SRFPROG) -t 'soc(XDS-0000000S)' -e all -p 'epfw(0)' -v rb -f $(CONTIKI_PROJECT).hex -a 0x0