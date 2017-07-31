BOARD ?= nrf51_blenano
CONF_FILE = prj.conf

KBUILD_KCONFIG = $(PWD)/Kconfig
export KBUILD_KCONFIG

include $(ZEPHYR_BASE)/Makefile.inc

flash: flash_nrf51

flash_nrf51:
	openocd -f interface/stlink-v2.cfg -f target/nrf51.cfg -c "program outdir/nrf51_blenano/zephyr.elf verify reset exit"
