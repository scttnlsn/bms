#include <logging/sys_log.h>

#include "bms.h"
#include "bq769x0.h"

#define NUM_CELLS 4

static uint8_t cells[NUM_CELLS] = { 0, 1, 2, 4, };
static uint16_t cell_voltages[NUM_CELLS];

int bms_init(void) {
  SYS_LOG_INF("initializing...");

  int ret = bq769x0_init();
  if (ret < 0) {
    SYS_LOG_ERR("failed to initialize BQ769x0");
    return ret;
  }

  ret = bq769x0_boot("GPIO_0", 21);
  if (ret < 0) {
    SYS_LOG_ERR("failed to boot BQ769x0");
    return ret;
  }

  bq769x0_config_t bq769x0_config;
  bq769x0_config.uvp = 3000;
  bq769x0_config.ovp = 3550;

  ret = bq769x0_configure(bq769x0_config);
  if (ret < 0) {
    SYS_LOG_ERR("failed to configure BQ769x0");
    return ret;
  }

  SYS_LOG_INF("initialized");
  return 0;
}

void bms_measure(void) {
  for (int i = 0; i < NUM_CELLS; i++) {
    cell_voltages[i] = bq769x0_read_cell_voltage(cells[i]);
  }
}

uint16_t *bms_cell_voltages(void) {
  return cell_voltages;
}
