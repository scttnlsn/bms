# BMS

Firmware for controlling [TI BQ769x0](http://www.ti.com/product/BQ76920) battery monitors.  Designed to run on the [Nordic nRF51822](https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF51822) ARM Cortex-M0 MCU.

Requirements:

* [GNU ARM Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)
* [OpenOCD](http://openocd.org)
* [Zephyr RTOS](https://www.zephyrproject.org) (included as submodule)

## Getting started

Setup:

* `git submodule update --init`
* `source env.sh`

Build:

* `make`
* `make flash`

## Configuration

There are various configuration options which can be specified via Kconfig.  Run `make menuconfig` to browse the available options and override any of the default values.

* `CONFIG_BMS_BOOT_DEVICE` (default: `GPIO_0`)
* `CONFIG_BMS_BOOT_PIN` (default: `21`)
* `CONFIG_BMS_ALERT_DEVICE` (default: `GPIO_0`)
* `CONFIG_BMS_ALERT_PIN` (default: `22`)
* `CONFIG_BMS_BLINK_DEVICE` (default: `GPIO_0`)
* `CONFIG_BMS_BLINK_PIN` (default: `18`)

* `CONFIG_BMS_OVP_ENABLE` (default: `3550` mV)
* `CONFIG_BMS_OVP_DISABLE` (default: `3400` mV)
* `CONFIG_BMS_UVP_ENABLE` (default: `3300` mV)
* `CONFIG_BMS_UVP_DISABLE` (default: `3100` mV)

* `CONFIG_BMS_SCD_DELAY` (default: `10000` ms)
* `CONFIG_BMS_OCD_DELAY` (default: `10000` ms)
