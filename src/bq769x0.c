#include <gpio.h>
#include <i2c.h>
#include <logging/sys_log.h>

#include "bq769x0.h"

static struct device *i2c;

int bq769x0_init(void) {
  SYS_LOG_INF("initializing...");

  // TODO: make binding a config var
  i2c = device_get_binding("I2C_0");

  if (i2c == NULL) {
    SYS_LOG_ERR("could not find I2C_0");
    return -EINVAL;
  }

  SYS_LOG_INF("initialized");

  return 0;
}

int bq769x0_boot(char *boot_port, int boot_pin) {
  SYS_LOG_INF("booting...");

  struct device *gpio = device_get_binding(boot_port);

  if (gpio == NULL) {
    SYS_LOG_ERR("could not find %s", boot_port);
    return -EINVAL;
  }

  gpio_pin_configure(gpio, boot_pin, GPIO_DIR_OUT);
  gpio_pin_write(gpio, boot_pin, 1);

  // datasheet says wait at least 2ms
  k_sleep(5);

  gpio_pin_configure(gpio, boot_pin, GPIO_DIR_IN);

  // datasheets says wait at least 10ms
  k_sleep(20);

  SYS_LOG_INF("booted");
}

int bq769x0_configure(void) {
  SYS_LOG_INF("configuring...");

  // test I2C communication
  u8_t cfg;

  if (i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_CFG, BQ769X0_CFG_VALUE) < 0) {
    SYS_LOG_ERR("failed to write CFG register");
    return -EIO;
  }

  if (i2c_reg_read_byte(i2c, 0x08, 0x19, &cfg) < 0) {
    SYS_LOG_ERR("failed to read CFG register");
    return -EIO;
  }

  if (cfg != BQ769X0_CFG_VALUE) {
    SYS_LOG_ERR("invalid CFG value");
    return -EINVAL;
  }

  SYS_LOG_INF("configured");

  return 0;
}
