#include <gpio.h>
#include <i2c.h>
#include <logging/sys_log.h>

#include "bq769x0.h"

#define BQ769X0_ADDR 0x08
#define BQ769X0_CFG_VALUE 0x19

// registers

#define BQ769X0_REG_SYS_STAT 0x00
#define BQ769X0_REG_CELLBAL1 0x01
#define BQ769X0_REG_SYS_CTRL1 0x04
#define BQ769X0_REG_SYS_CTRL2 0x05
#define BQ769X0_REG_PROTECT1 0x06
#define BQ769X0_REG_PROTECT2 0x07
#define BQ769X0_REG_PROTECT3 0x08
#define BQ769X0_REG_OV_TRIP 0x09
#define BQ769X0_REG_UV_TRIP 0x0A
#define BQ769X0_REG_CFG 0x0B
#define BQ769X0_REG_VC1_HI 0x0C
#define BQ769X0_REG_BAT_HI 0x2A
#define BQ769X0_REG_BAT_LO 0x2B
#define BQ769X0_REG_CC_HI 0x32
#define BQ769X0_REG_CC_LO 0x33
#define BQ769X0_REG_ADCGAIN1 0x50
#define BQ769X0_REG_ADCOFFSET 0x51
#define BQ769X0_REG_ADCGAIN2 0x59

// bits

#define BQ769X0_REG_CTRL1_ADC_EN 4

#define BQ769X0_REG_STAT_OCD 0
#define BQ769X0_REG_STAT_SCD 1
#define BQ769X0_REG_STAT_OV 2
#define BQ769X0_REG_STAT_UV 3
#define BQ769X0_REG_STAT_OVRD_ALERT 4
#define BQ769X0_REG_STAT_DEVICE_XREADY 5
#define BQ769X0_REG_STAT_CC_READY 7

#define BQ769X0_REG_CTRL2_CHG_ON 0
#define BQ769X0_REG_CTRL2_DSG_ON 1
#define BQ769X0_REG_CTRL2_CC_ONESHOT 5
#define BQ769X0_REG_CTRL2_CC_EN 6
#define BQ769X0_REG_CTRL2_DELAY_DIS 7

#define BQ769X0_REG_PROTECT1_SCD_T 0
#define BQ769X0_REG_PROTECT1_SCD_D 3
#define BQ769X0_REG_PROTECT1_RSNS 7

#define BQ769X0_REG_PROTECT2_OCD_T 0
#define BQ769X0_REG_PROTECT2_OCD_D 4

#define BQ769X0_REG_PROTECT3_OV_D 4
#define BQ769X0_REG_PROTECT3_UV_D 6

static struct device *i2c;

static int8_t adc_offset;
static uint16_t adc_gain;

int bq769x0_init(char *i2c_device) {
  int rc;

  SYS_LOG_INF("initializing...");

  i2c = device_get_binding(i2c_device);

  if (i2c == NULL) {
    SYS_LOG_ERR("could not find %s", i2c_device);
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

  rc = i2c_configure(i2c, i2c_cfg.raw);
  if (rc < 0) {
    SYS_LOG_ERR("could not configure I2C_0");
    return rc;
  }

  SYS_LOG_INF("initialized");
  return 0;
}

int bq769x0_boot(char *boot_device, int boot_pin) {
  SYS_LOG_INF("booting...");

  struct device *gpio = device_get_binding(boot_device);

  if (gpio == NULL) {
    SYS_LOG_ERR("could not find %s", boot_device);
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

  int rc;

  // test I2C communication
  uint8_t cfg;

  rc = i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_CFG, BQ769X0_CFG_VALUE);
  if (rc < 0) {
    SYS_LOG_ERR("failed to write CFG register");
    return rc;
  }

  rc = i2c_reg_read_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_CFG, &cfg);
  if (rc < 0) {
    SYS_LOG_ERR("failed to read CFG register");
    return rc;
  }

  if (cfg != BQ769X0_CFG_VALUE) {
    SYS_LOG_ERR("invalid CFG value");
    return -EINVAL;
  }

  // enable ADC
  uint8_t adc_en = 1 << BQ769X0_REG_CTRL1_ADC_EN;
  rc = i2c_reg_update_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_SYS_CTRL1, adc_en, adc_en);
  if (rc < 0) {
    SYS_LOG_ERR("failed to enable ADC");
    return rc;
  }

  // enable continous current monitoring
  uint8_t cc_en = 1 << BQ769X0_REG_CTRL2_CC_EN;
  rc = i2c_reg_update_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_SYS_CTRL2, cc_en, cc_en);
  if (rc < 0) {
    SYS_LOG_ERR("failed to enable CC");
    return rc;
  }

  // read ADC offset
  rc = i2c_reg_read_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_ADCOFFSET, &adc_offset);
  if (rc < 0) {
    SYS_LOG_ERR("failed to read ADCOFFSET");
    return rc;
  }

  // read ADC gain
  uint8_t adc_gain1;
  uint8_t adc_gain2;
  rc = i2c_reg_read_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_ADCGAIN1, &adc_gain1);
  if (rc < 0) {
    SYS_LOG_ERR("failed to read ADCGAIN1");
    return rc;
  }
  rc = i2c_reg_read_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_ADCGAIN2, &adc_gain2);
  if (rc < 0) {
    SYS_LOG_ERR("failed to read ADCGAIN2");
    return rc;
  }
  adc_gain = 365 + (((adc_gain1 & 0b00001100) << 1) | ((adc_gain2 & 0b11100000) >> 5));

  // clear all alerts
  rc = i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_SYS_STAT, 0b10111111);
  if (rc < 0) {
    SYS_LOG_ERR("failed to clear alerts");
    return rc;
  }

  // set under-voltage protection
  uint8_t uvp = ((((config.uvp - adc_offset) * 1000 / adc_gain) >> 4) & 0xFF) + 1;
  rc = i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_UV_TRIP, uvp);
  if (rc < 0) {
    SYS_LOG_ERR("failed to write UV_TRIP");
    return rc;
  }

  // set over-voltage protection
  uint8_t ovp = (((config.ovp - adc_offset) * 1000 / adc_gain) >> 4) & 0xFF;
  rc = i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_OV_TRIP, ovp);
  if (rc < 0) {
    SYS_LOG_ERR("failed to write OV_TRIP");
    return rc;
  }

  // set short-circuit protection
  uint8_t protect1 = 0;
  protect1 |= (1 << BQ769X0_REG_PROTECT1_RSNS); // set RSNS = 1
  protect1 |= (0x2 << BQ769X0_REG_PROTECT1_SCD_D); // 200uS delay
  protect1 |= (0x7 << BQ769X0_REG_PROTECT1_SCD_T); // 200mV across 0.005ohm = 40A
  rc = i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_PROTECT1, protect1);
  if (rc < 0) {
    SYS_LOG_ERR("failed to write PROTECT1");
    return rc;
  }

  // set over-current discharge protection
  uint8_t protect2 = 0;
  protect2 |= (0x0 << BQ769X0_REG_PROTECT2_OCD_D); // 8ms delay
  protect2 |= (0x6 << BQ769X0_REG_PROTECT2_OCD_T); // 50mV across 0.005ohm = 10A
  rc = i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_PROTECT2, protect2);
  if (rc < 0) {
    SYS_LOG_ERR("failed to write PROTECT2");
    return rc;
  }

  // set uv and ov delay
  uint8_t protect3 = 0;
  protect3 |= (0x1 << BQ769X0_REG_PROTECT3_UV_D); // 4s
  protect3 |= (0x1 << BQ769X0_REG_PROTECT3_OV_D); // 2s
  rc = i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_PROTECT3, protect3);
  if (rc < 0) {
    SYS_LOG_ERR("failed to write PROTECT3");
    return rc;
  }

  SYS_LOG_INF("configured");
  return 0;
}

int bq769x0_read_voltage(int cell, uint16_t *voltage) {
  int rc;

  uint8_t reg = BQ769X0_REG_VC1_HI + cell * 2;
  uint8_t buffer[2];

  rc = i2c_burst_read(i2c, BQ769X0_ADDR, reg, buffer, 2);
  if (rc < 0) {
    SYS_LOG_ERR("failed to read cell %d voltage", cell);
    return rc;
  }

  uint16_t adc_value = ((buffer[0] & 0b00111111) << 8) | buffer[1];
  *voltage = adc_value * adc_gain / 1000 + adc_offset;

  return 0;
}

int bq769x0_read_current(int16_t *current) {
  int rc;

  // read CC
  uint8_t msb;
  uint8_t lsb;

  rc = i2c_reg_read_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_CC_HI, &msb);
  if (rc < 0) {
    SYS_LOG_ERR("failed to read CC_HI");
    return rc;
  }

  rc = i2c_reg_read_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_CC_LO, &lsb);
  if (rc < 0) {
    SYS_LOG_ERR("failed to read CC_LO");
    return rc;
  }

  uint16_t adc_value = (msb << 8) | lsb;
  *current = (int16_t) adc_value * 8.44 / 5.0;

  // clear CC_READY
  uint8_t cc_ready = 1 << BQ769X0_REG_STAT_CC_READY;
  rc = i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_SYS_STAT, cc_ready);
  if (rc < 0) {
    SYS_LOG_ERR("failed to clear CC_READY");
    return rc;
  }

  return 0;
}

int bq769x0_read_status(uint8_t *status) {
  int rc;

  rc = i2c_reg_read_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_SYS_STAT, status);
  if (rc < 0) {
    SYS_LOG_ERR("failed to read status");
    return rc;
  }

  return 0;
}

int bq769x0_clear_status(uint8_t bit) {
  int rc;

  uint8_t mask = 1 << bit;
  rc = i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_SYS_STAT, mask);
  if (rc < 0) {
    SYS_LOG_ERR("failed to clear status: %d", mask);
    return rc;
  }

  return 0;
}

uint8_t bq769x0_status_error(uint8_t status) {
  // mask out CC_READY (not considered an error)
  return status & 0b01111111;
}

uint8_t bq769x0_cc_ready(uint8_t status) {
  return status & 0b10000000;
}

int bq769x0_enable_discharging(void) {
  int rc;

  uint8_t dsg_on = 1 << BQ769X0_REG_CTRL2_DSG_ON;
  rc = i2c_reg_update_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_SYS_CTRL2, dsg_on, dsg_on);
  if (rc < 0) {
    SYS_LOG_ERR("failed to enable discharging");
    return rc;
  }

  return 0;
}

int bq769x0_enable_charging(void) {
  int rc;

  uint8_t chg_on = 1 << BQ769X0_REG_CTRL2_CHG_ON;
  rc = i2c_reg_update_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_SYS_CTRL2, chg_on, chg_on);
  if (rc < 0) {
    SYS_LOG_ERR("failed to enable charging");
    return rc;
  }

  return 0;
}

int bq769x0_balance_cell(int8_t cell) {
  int rc;

  if (cell == -1) {
    rc = i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_CELLBAL1, 0);
    if (rc < 0) {
      SYS_LOG_ERR("failed to disable cell balancing");
      return rc;
    }
  } else {
    rc = i2c_reg_write_byte(i2c, BQ769X0_ADDR, BQ769X0_REG_CELLBAL1, 1 << cell);
    if (rc < 0) {
      SYS_LOG_ERR("failed to enable cell balancing");
      return rc;
    }
  }

  return 0;
}
