#include <gpio.h>
#include <misc/printk.h>
#include <logging/sys_log.h>
#include <zephyr.h>

#include "ble.h"
#include "bms.h"

#define STACKSIZE 1024
#define PRIORITY 5

// blinker thread

K_THREAD_STACK_DEFINE(blinker_thread_stack_area, STACKSIZE);
static struct k_thread blinker_thread_data;

void blinker_thread(void *a, void *b, void *c) {
  struct device *gpio = device_get_binding("GPIO_0");

  gpio_pin_configure(gpio, 18, GPIO_DIR_OUT);

  while (1) {
    gpio_pin_write(gpio, 18, 1);
    k_sleep(250);
    gpio_pin_write(gpio, 18, 0);
    k_sleep(250);
  }
}

// bms thread

K_THREAD_STACK_DEFINE(bms_thread_stack_area, STACKSIZE);
static struct k_thread bms_thread_data;

void bms_thread(void *a, void *b, void *c) {
  int rc = bms_init();

  if (rc < 0) {
    SYS_LOG_INF("failed to initialize BMS");
  }

  while (1) {
    bms_measure();
    k_sleep(500);
  }
}

// serial logger thread

K_THREAD_STACK_DEFINE(logger_thread_stack_area, STACKSIZE);
static struct k_thread logger_thread_data;

void logger_thread(void *a, void *b, void *c) {
  uint16_t *voltages;
  int16_t current;

  while (1) {
    voltages = bms_cell_voltages();
    current = bms_pack_current();

    printk("mV=%d, %d, %d, %d\n", voltages[0], voltages[1], voltages[2], voltages[3]);
    printk("mA=%d\n", current);

    uint16_t values[5] = {
      voltages[0],
      voltages[1],
      voltages[2],
      voltages[3],
      current
    };

    ble_notify((uint8_t *) values, 10);

    k_sleep(2000);
  }
}

void main(void) {
  SYS_LOG_INF("starting...");

  ble_init();

  k_thread_create(
    &blinker_thread_data, blinker_thread_stack_area,
    K_THREAD_STACK_SIZEOF(blinker_thread_stack_area),
    blinker_thread,
    NULL, NULL, NULL,
    PRIORITY, 0, K_NO_WAIT);

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
