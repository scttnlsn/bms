BOARD ?= nrf51_blenano
CONF_FILE = prj.conf

include $(ZEPHYR_BASE)/Makefile.inc

flash:
	openocd -f interface/stlink-v2.cfg -f target/nrf51.cfg -c "program outdir/nrf51_blenano/zephyr.elf verify reset exit"
