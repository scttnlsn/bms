#include <gpio.h>
#include <misc/printk.h>
#include <logging/sys_log.h>
#include <zephyr.h>

#include "ble.h"
#include "blinker.h"
#include "bms.h"

#define STACKSIZE 1024
#define PRIORITY 5

typedef struct __attribute__((__packed__)) {
  uint16_t voltages[BMS_NUM_CELLS];
  int16_t current;
  int32_t charge;
  uint8_t soc;
} bms_data_t;

static bms_data_t bms_data;

// bms thread

K_THREAD_STACK_DEFINE(bms_thread_stack_area, STACKSIZE);
static struct k_thread bms_thread_data;

void bms_thread(void *a, void *b, void *c) {
  while (1) {
    bms_update();
    k_sleep(200);
  }
}

// serial logger thread

K_THREAD_STACK_DEFINE(logger_thread_stack_area, STACKSIZE);
static struct k_thread logger_thread_data;

void logger_thread(void *a, void *b, void *c) {
  while (1) {
    bms_cell_voltages(bms_data.voltages);
    bms_data.current = bms_current();
    bms_data.charge = bms_charge();
    bms_data.soc = bms_soc();

    printk("mV=%d, %d, %d, %d\n", bms_data.voltages[0], bms_data.voltages[1], bms_data.voltages[2], bms_data.voltages[3]);
    printk("mA=%d\n", bms_data.current);
    printk("charge=%d\n", bms_data.charge);
    printk("soc=%d\n", bms_data.soc);

    ble_notify((uint8_t *) &bms_data, sizeof(bms_data));

    k_sleep(2000);
  }
}

// serial logger thread

K_THREAD_STACK_DEFINE(flasher_thread_stack_area, STACKSIZE);
static struct k_thread flasher_thread_data;

void flasher_thread(void *a, void *b, void *c) {
  while (1) {
    blinker_flash();
    k_sleep(30000);
  }
}

void main(void) {
  SYS_LOG_INF("starting...");

  ble_init();
  blinker_init(CONFIG_BMS_BLINK_DEVICE, CONFIG_BMS_BLINK_PIN);

  int rc = bms_init();
  if (rc < 0) {
    SYS_LOG_INF("failed to initialize BMS");
    return;
  }

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

  k_thread_create(
    &flasher_thread_data, flasher_thread_stack_area,
    K_THREAD_STACK_SIZEOF(flasher_thread_stack_area),
    flasher_thread,
    NULL, NULL, NULL,
    PRIORITY, 0, K_NO_WAIT);

  SYS_LOG_INF("started");
}

void _SysFatalErrorHandler(unsigned int reason, const NANO_ESF *pEsf) {
  blinker_on();
  while (1);
}
