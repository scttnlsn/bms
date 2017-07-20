#include <gpio.h>
#include <misc/printk.h>
#include <logging/sys_log.h>
#include <zephyr.h>

#include "bms.h"

#define STACKSIZE 1024
#define PRIORITY 5

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

void main(void) {
  SYS_LOG_INF("starting...");

  k_thread_create(
    &blinker_thread_data, blinker_thread_stack_area,
    K_THREAD_STACK_SIZEOF(blinker_thread_stack_area),
    blinker_thread,
    NULL, NULL, NULL,
    PRIORITY, 0, K_NO_WAIT);

  bms_init();

  SYS_LOG_INF("started");
}
