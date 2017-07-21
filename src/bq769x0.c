#include <gpio.h>
#include <i2c.h>
#include <logging/sys_log.h>

#include "bq769x0.h"

static struct device *i2c;

static int8_t adc_offset;
static uint16_t adc_gain;

int bq769x0_init(void) {
  SYS_LOG_INF("initializing...");

  // TODO: make binding a config var
  i2c = device_get_binding("I2C_0");

  if (i2c == NULL) {
    SYS_LOG_ERR("could not find I2C_0");
    return -EINVAL;
  }

  static union dev_config i2c_cfg = {
    .raw = 0,
    .bits = {
      .use_10_bit_addr = 0,
      .is_master_device = 1,
      .speed = I2C_SPEED_STANDARD,
    },
  };

  if (i2c_configure(i2c, i2c_cfg.raw)) {
    SYS_LOG_ERR("could not configure I2C_0");
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
  return 0;
}

int bq769x0_configure(bq769x0_config_t config) {
  SYS_LOG_INF("configuring...");

  // test I2C communication
  uint8_t cfg;

  if (i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_CFG, BQ769X0_CFG_VALUE) < 0) {
    SYS_LOG_ERR("failed to write CFG register");
    return -EIO;
  }

  if (i2c_reg_read_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_CFG, &cfg) < 0) {
    SYS_LOG_ERR("failed to read CFG register");
    return -EIO;
  }

  if (cfg != BQ769X0_CFG_VALUE) {
    SYS_LOG_ERR("invalid CFG value");
    return -EINVAL;
  }

  // enable ADC
  if (i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_SYS_CTRL1, 0b10000) < 0) {
    SYS_LOG_ERR("failed to enable ADC");
    return -EIO;
  }

  // clear all alerts
  if (i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_SYS_STAT, 0b11111111) < 0) {
    SYS_LOG_ERR("failed to clear alerts");
    return -EIO;
  }

  // disable continous current monitoring
  if (i2c_reg_update_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_SYS_CTRL2, 1 << BQ769X0_REG_CTRL2_CC_EN, 0) < 0) {
    SYS_LOG_ERR("failed disable CC");
    return -EIO;
  }

  // read ADC offset
  if (i2c_reg_read_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_ADCOFFSET, &adc_offset) < 0) {
    SYS_LOG_ERR("failed to read ADCOFFSET");
    return -EIO;
  }

  // read ADC gain
  uint8_t adc_gain1;
  uint8_t adc_gain2;
  if (i2c_reg_read_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_ADCGAIN1, &adc_gain1) < 0) {
    SYS_LOG_ERR("failed to read ADCGAIN1");
    return -EIO;
  }
  if (i2c_reg_read_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_ADCGAIN2, &adc_gain2) < 0) {
    SYS_LOG_ERR("failed to read ADCGAIN2");
    return -EIO;
  }
  adc_gain = 365 + (((adc_gain1 & 0b00001100) << 1) | ((adc_gain2 & 0b11100000) >> 5));

  // set under-voltage protection
  uint8_t uvp = ((((config.uvp - adc_offset) * 1000 / adc_gain) >> 4) & 0xFF) + 1;
  if (i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_UV_TRIP, uvp) < 0) {
    SYS_LOG_ERR("failed to write UV_TRIP");
    return -EIO;
  }

  // set over-voltage protection
  uint8_t ovp = (((config.ovp - adc_offset) * 1000 / adc_gain) >> 4) & 0xFF;
  if (i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_OV_TRIP, ovp) < 0) {
    SYS_LOG_ERR("failed to write OV_TRIP");
    return -EIO;
  }

  // set short-circuit protection
  uint8_t protect1 = 0;
  protect1 |= (1 << BQ769X0_REG_PROTECT1_RSNS); // set RSNS = 1
  protect1 |= (0x2 << BQ769X0_REG_PROTECT1_SCD_D); // 200uS delay
  protect1 |= (0x7 << BQ769X0_REG_PROTECT1_SCD_T); // 200mV across 0.005ohm = 40A
  if (i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_PROTECT1, protect1) < 0) {
    SYS_LOG_ERR("failed to write PROTECT1");
    return -EIO;
  }

  // set over-current discharge protection
  uint8_t protect2 = 0;
  protect2 |= (0x0 << BQ769X0_REG_PROTECT2_OCD_D); // 8ms delay
  protect2 |= (0x6 << BQ769X0_REG_PROTECT2_OCD_T); // 50mV across 0.005ohm = 10A
  if (i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_PROTECT2, protect2) < 0) {
    SYS_LOG_ERR("failed to write PROTECT2");
    return -EIO;
  }

  // set uv and ov delay
  uint8_t protect3 = 0;
  protect3 |= (0x1 << BQ769X0_REG_PROTECT3_UV_D); // 4s
  protect3 |= (0x1 << BQ769X0_REG_PROTECT3_OV_D); // 2s
  if (i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_PROTECT3, protect3) < 0) {
    SYS_LOG_ERR("failed to write PROTECT3");
    return -EIO;
  }

  SYS_LOG_INF("configured");
  return 0;
}
