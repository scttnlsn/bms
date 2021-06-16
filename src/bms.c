#include <gpio.h>
#include <logging/sys_log.h>
#include <string.h>
#include <zephyr.h>

#include "bms.h"
#include "bq769x0.h"

static uint8_t status;
static uint8_t cells[BMS_NUM_CELLS] = { 0, 1, 2, 4, };
static uint16_t cell_voltages[BMS_NUM_CELLS];
static int16_t current; // most recent 250ms current average
static u64_t capacity = BMS_NOMINAL_CAPACITY;
static s64_t charge = -1; // cumulative mA seconds
static uint8_t ov = 0;
static uint8_t uv = 0;
static uint8_t scd = 0;
static s64_t scd_tick = 0;
static uint8_t ocd = 0;
static s64_t ocd_tick = 0;

void bms_handle_error(uint8_t status);
void bms_interpolate_charge(void);

static struct gpio_callback alert_cb;

static void bms_alert(struct device *gpiob, struct gpio_callback *cb, u32_t pins) {
  // TODO
}

int bms_init(void) {
  int rc;

  SYS_LOG_INF("initializing...");

  // setup alert interrupt
  struct device *gpio = device_get_binding(CONFIG_BMS_ALERT_DEVICE);
  if (gpio == NULL) {
    SYS_LOG_ERR("failed to get binding for %s", CONFIG_BMS_ALERT_DEVICE);
    return -EINVAL;
  }

  gpio_pin_configure(gpio, CONFIG_BMS_ALERT_PIN, GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH | GPIO_PUD_PULL_DOWN);
  gpio_init_callback(&alert_cb, bms_alert, BIT(CONFIG_BMS_ALERT_PIN));
  gpio_add_callback(gpio, &alert_cb);
  gpio_pin_enable_callback(gpio, CONFIG_BMS_ALERT_PIN);

  rc = bq769x0_init(CONFIG_BMS_I2C_DEVICE);
  if (rc < 0) {
    SYS_LOG_ERR("failed to initialize BQ769x0");
    return rc;
  }

  rc = bq769x0_boot(CONFIG_BMS_BOOT_DEVICE, CONFIG_BMS_BOOT_PIN);
  if (rc < 0) {
    SYS_LOG_ERR("failed to boot BQ769x0");
    return rc;
  }

  bq769x0_config_t bq769x0_config;
  bq769x0_config.ovp = CONFIG_BMS_OVP_ENABLE;
  bq769x0_config.uvp = CONFIG_BMS_UVP_ENABLE;

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

// should be called at least every 250ms for accurate coulomb counting
int bms_update(void) {
  int rc;

  rc = bq769x0_read_status(&status);
  if (rc < 0) {
    return rc;
  }

  uint8_t error = bq769x0_status_error(status);
  if (error) {
    bms_handle_error(error);
  }

  for (int i = 0; i < BMS_NUM_CELLS; i++) {
    bq769x0_read_voltage(cells[i], &cell_voltages[i]);
  }

  if (charge == -1) {
    bms_interpolate_charge();
  }

  uint8_t cc_ready = bq769x0_cc_ready(status);
  if (cc_ready) {
    bq769x0_read_current(&current);
    charge += (current / 4); // CC updated every 250ms
  }

  if (scd) {
    if (k_uptime_get() - scd_tick > CONFIG_BMS_SCD_DELAY) {
      bq769x0_clear_status(BQ769X0_STATUS_SCD);
      bq769x0_enable_charging();
      bq769x0_enable_discharging();
    } else {
      return 0;
    }
  }

  if (ocd) {
    if (k_uptime_get() - ocd_tick > CONFIG_BMS_OCD_DELAY) {
      bq769x0_clear_status(BQ769X0_STATUS_OCD);
      bq769x0_enable_charging();
      bq769x0_enable_discharging();
    } else {
      return 0;
    }
  }

  if (ov) {
    // check to see if voltages have fallen enough to disable OVP
    uint8_t disable = 1;

    for (int i = 0; i < BMS_NUM_CELLS; i++) {
      disable = disable & (cell_voltages[i] <= CONFIG_BMS_OVP_DISABLE);
    }

    if (disable) {
      bq769x0_clear_status(BQ769X0_STATUS_OV);
      bq769x0_enable_charging();
    }
  }

  if (uv) {
    // check to see if voltages have risen enough to disable UVP
    uint8_t disable = 1;

    for (int i = 0; i < BMS_NUM_CELLS; i++) {
      disable = disable & (cell_voltages[i] >= CONFIG_BMS_UVP_DISABLE);
    }

    if (disable) {
      bq769x0_clear_status(BQ769X0_STATUS_UV);
      bq769x0_enable_discharging();
    }
  }

  return 0;
}

uint8_t bms_status() {
  return status & 0xF; // only include: UV | OV | SCD | OCD |
}

int bms_cell_voltages(uint16_t *voltages) {
  int len = sizeof(cell_voltages);
  memcpy(voltages, cell_voltages, len);
  return len;
}

int16_t bms_current(void) {
  return current;
}

int32_t bms_charge(void) {
  return charge;
}

uint8_t bms_soc(void) {
  return (capacity + charge) * 100 / capacity;
}

// helpers

void bms_handle_error(uint8_t status) {
  SYS_LOG_INF("error status: %d", status);

  if (status & 0b00000100) {
    // over voltage
    if (!ov) {
      SYS_LOG_ERR("over voltage limit");
      ov = 1;
      charge = 0;
    }
  } else {
    ov = 0;
  }

  if (status & 0b00001000) {
    // under voltage
    if (!uv) {
      SYS_LOG_ERR("under voltage limit");
      uv = 1;
      capacity = -charge;

      // TODO: persist updated capacity
    }

    // TODO: probably shut down the MCU here as well since it continues to draw current
  } else {
    uv = 0;
  }

  if (status & 0b00000010) {
    // short circuit
    if (!scd) {
      SYS_LOG_ERR("short circuit");
      scd_tick = k_uptime_get();
    }
    scd = 1;
  } else {
    scd = 0;
  }

  if (status & 0b00000001) {
    // overcurrent discharge
    if (!ocd) {
      SYS_LOG_ERR("overcurrent discharge");
      ocd_tick = k_uptime_get();
    }
    ocd = 1;
  } else {
    ocd = 0;
  }
}

void bms_interpolate_charge(void) {
  SYS_LOG_INF("interpolating charge");

  uint32_t average_cell_voltage = 0;
  for (int i = 0; i < BMS_NUM_CELLS; i++) {
    average_cell_voltage += cell_voltages[i];
  }

  average_cell_voltage /= BMS_NUM_CELLS;

  uint32_t mv_range = CONFIG_BMS_OVP_ENABLE - CONFIG_BMS_UVP_ENABLE;
  u64_t capacity_per_mv = BMS_NOMINAL_CAPACITY / mv_range;
  uint32_t mv_offset = CONFIG_BMS_OVP_ENABLE - average_cell_voltage;

  charge = -(capacity_per_mv * mv_offset);
}
