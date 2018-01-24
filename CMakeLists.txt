set(BOARD nrf51_blenano)
set(KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Kconfig)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(NONE)

target_sources(app PRIVATE
  src/main.c
  src/bq769x0.c
  src/bms.c
  src/ble.c
  src/blinker.c
  src/utils.c
)
