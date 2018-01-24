build:
	cd build && make

flash:
	openocd -f interface/stlink-v2.cfg -f target/nrf51.cfg -c "program ./build/zephyr/zephyr.elf verify reset exit"

menuconfig:
	cd build && make menuconfig

.PHONY: build flash menuconfig
