#include <gpio.h>
#include <misc/printk.h>
#include <logging/sys_log.h>
#include <zephyr.h>

#include "ble.h"
#include "blinker.h"
#include "bms.h"

#define STACKSIZE 1024
#define PRIORITY 5

// bms thread

K_THREAD_STACK_DEFINE(bms_thread_stack_area, STACKSIZE);
static struct k_thread bms_thread_data;

void bms_thread(void *a, void *b, void *c) {
  int rc = bms_init();

  if (rc < 0) {
    SYS_LOG_INF("failed to initialize BMS");
  }

  while (1) {
    bms_update();
    k_sleep(200);
  }
}

// serial logger thread

K_THREAD_STACK_DEFINE(logger_thread_stack_area, STACKSIZE);
static struct k_thread logger_thread_data;

void logger_thread(void *a, void *b, void *c) {
  uint16_t *voltages;
  int16_t current;
  int32_t charge;

  while (1) {
    voltages = bms_cell_voltages();
    current = bms_current();
    charge = bms_charge();

    printk("mV=%d, %d, %d, %d\n", voltages[0], voltages[1], voltages[2], voltages[3]);
    printk("mA=%d\n", current);
    printk("charge=%d\n", charge);

    uint16_t values[5] = {
      voltages[0],
      voltages[1],
      voltages[2],
      voltages[3],
      current
    };

    ble_notify((uint8_t *) values, 10);
    blinker_flash();

    k_sleep(2000);
  }
}

void main(void) {
  SYS_LOG_INF("starting...");

  ble_init();
  blinker_init(CONFIG_BMS_BLINK_DEVICE, CONFIG_BMS_BLINK_PIN);

  k_thread_create(
    &bms_thread_data, bms_thread_stack_area,
    K_THREAD_STACK_SIZEOF(bms_thread_stack_area),
    bms_thread,
    NULL, NULL, NULL,
    PRIORITY, 0, K_NO_WAIT);

  k_thread_create(
    &logger_thread_data, logger_thread_stack_area,
    K_THREAD_STACK_SIZEOF(logger_thread_stack_area),
    logger_thread,
    NULL, NULL, NULL,
    PRIORITY, 0, K_NO_WAIT);

  SYS_LOG_INF("started");
}
