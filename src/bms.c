#include <gpio.h>
#include <logging/sys_log.h>
#include <zephyr.h>

#include "bms.h"
#include "bq769x0.h"

#define NUM_CELLS 4

static uint8_t cells[NUM_CELLS] = { 0, 1, 2, 4, };
static uint16_t cell_voltages[NUM_CELLS];
static int16_t pack_current;

static struct gpio_callback alert_cb;

// state
static uint32_t error_tick = 0;
static uint8_t alert_triggered = 0;
static uint8_t ov = 0;
static uint8_t uv = 0;

void bms_handle_alert(void);
void bms_handle_error(uint8_t status);

static void bms_alert(struct device *gpiob, struct gpio_callback *cb, u32_t pins) {
  alert_triggered = 1;
}

int bms_init(void) {
  int rc;

  SYS_LOG_INF("initializing...");

  // setup alert interrupt
  struct device *gpio = device_get_binding("GPIO_0");
  if (gpio == NULL) {
    SYS_LOG_ERR("failed to get binding for %s", "GPIO_0");
    return -EINVAL;
  }

  gpio_pin_configure(gpio, 22, GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH);
  gpio_init_callback(&alert_cb, bms_alert, BIT(22));
  gpio_add_callback(gpio, &alert_cb);
  gpio_pin_enable_callback(gpio, 22);

  rc = bq769x0_init();
  if (rc < 0) {
    SYS_LOG_ERR("failed to initialize BQ769x0");
    return rc;
  }

  rc = bq769x0_boot("GPIO_0", 21);
  if (rc < 0) {
    SYS_LOG_ERR("failed to boot BQ769x0");
    return rc;
  }

  bq769x0_config_t bq769x0_config;
  bq769x0_config.uvp = 3000;
  bq769x0_config.ovp = 3550;

  rc = bq769x0_configure(bq769x0_config);
  if (rc < 0) {
    SYS_LOG_ERR("failed to configure BQ769x0");
    return rc;
  }

  rc = bq769x0_balance_cell(-1);
  if (rc < 0) {
    return rc;
  }

  rc = bq769x0_enable_charging();
  if (rc < 0) {
    return rc;
  }

  rc = bq769x0_enable_discharging();
  if (rc < 0) {
    return rc;
  }

  SYS_LOG_INF("initialized");
  return 0;
}

void bms_measure(void) {
  if (alert_triggered) {
    alert_triggered = 0;

    bms_handle_alert();
  }

  for (int i = 0; i < NUM_CELLS; i++) {
    bq769x0_read_voltage(cells[i], &cell_voltages[i]);
  }

  bq769x0_read_current(&pack_current);
}

uint16_t *bms_cell_voltages(void) {
  return cell_voltages;
}

int16_t bms_pack_current(void) {
  return pack_current;
}

// helpers

void bms_handle_alert(void) {
  uint8_t status;

  if (bq769x0_read_status(&status) < 0) {
    return;
  }

  // mask out CC_READY (not considered an error)
  uint8_t error = status & 0b01111111;

  if (error) {
    bms_handle_error(error);
  }
}

void bms_handle_error(uint8_t status) {
  SYS_LOG_INF("error: %d", status);

  error_tick = k_cycle_get_32();

  if (status & 0b00000100) {
    // over voltage
    SYS_LOG_ERR("OVER VOLTAGE");
    ov = 1;
  }

  if (status & 0b00001000) {
    // under voltage
    SYS_LOG_ERR("UNDER VOLTAGE");
    uv = 1;

    // TODO: probably shut down the MCU here as well, how do we know when the re-enable?
  }

  if (status & 0b00000010) {
    // short circuit
    SYS_LOG_ERR("SHORT CIRCUIT");

    // TODO: wait some time before clearing this
    // currently requires a reset
  }

  if (status & 0b00000001) {
    // overcurrent discharge
    SYS_LOG_ERR("OVERCURRENT DISCHARGE");

    // TODO: wait some time before clearing this
    // currently requires a reset
  }
}
