# BMS

Integrated battery management system for 4-series li-ion packs (based on the [BQ76920](http://www.ti.com/product/BQ76920) from Texas Instruments).

![Board](hardware/board.jpg)

* programmable over/under voltage protection for each cell
* programmable short-circuit detection
* auto cell balancing
* cell voltage monitoring
* coulomb counting for accurate state-of-charge tracking
* status available over BLE

## Hardware

Full schematic and PCB layout available in the `hardware` directory.

* [TI BQ76920](http://www.ti.com/product/BQ76920) analog front-end
* [Nordic nRF51822](https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF51822) ARM Cortex-M host controller
* FETs for enabling or disabling charge/discharge
* shunt resistor for current measurement

## Firmware

The host controller firmware is based on the [Zephyr RTOS](https://www.zephyrproject.org).  It communicates with the BQ76920 via I2C and handles various error states that may occur in the BQ76920.  It also exposes a BLE peripheral for wireless monitoring.

### Build

Requirements:

* [GNU ARM Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)
* [OpenOCD](http://openocd.org)
* [Zephyr RTOS](https://www.zephyrproject.org) (included as submodule)

Setup:

* `git submodule update --init`
* `./setup.sh`

Compile and flash:

* `make`
* `make flash`

### Configuration

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
* current (mA) [2 bytes signed]
* charge used (mAs) [4 bytes signed]
* state of charge (percent) [1 byte]
