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
* `CONFIG_BMS_OVP_DISABLE` (default: `3350` mV)
* `CONFIG_BMS_UVP_ENABLE` (default: `3300` mV)
* `CONFIG_BMS_UVP_DISABLE` (default: `3100` mV)
* `CONFIG_BMS_SCD_DELAY` (default: `10000` ms)
* `CONFIG_BMS_OCD_DELAY` (default: `10000` ms)

## BLE Protocol

The BMS implements a Bluetooth Low Energy peripheral for monitoring cell voltages, current draw and state-of-charge.  All status data is part of a single characteristic.

Peripheral UUID: `8D9D7800-5B61-412A-AB71-5C7E0E559086`

Characteristic UUID: `8D9D7801-5B61-412A-AB71-5C7E0E559086`

Value structure (MSB to LSB):

* cell 1 (mV) [2 bytes]
* cell 2 (mV) [2 bytes]
* cell 3 (mV) [2 bytes]
* cell 4 (mV) [2 bytes]
* current (mA) [4 bytes signed]
* charge used (mAs) [4 bytes signed]
* state of charge (percent) [4 bytes]
