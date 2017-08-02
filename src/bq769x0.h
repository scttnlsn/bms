#ifndef __BQ769X0_H__
#define __BQ769X0_H__

#define BQ769X0_STATUS_OCD 0
#define BQ769X0_STATUS_SCD 1
#define BQ769X0_STATUS_OV 2
#define BQ769X0_STATUS_UV 3

typedef struct {
  uint16_t uvp; // mV
  uint16_t ovp; // mV
} bq769x0_config_t;

int bq769x0_init(char *i2c_device);
int bq769x0_boot(char *boot_device, int boot_pin);
int bq769x0_configure(bq769x0_config_t config);
int bq769x0_read_voltage(int cell_n, uint16_t *voltage);
int bq769x0_read_current(int16_t *current);
int bq769x0_read_status(uint8_t *status);
int bq769x0_clear_status(uint8_t bit);
uint8_t bq769x0_status_error(uint8_t status);
uint8_t bq769x0_cc_ready(uint8_t status);
int bq769x0_enable_discharging(void);
int bq769x0_enable_charging(void);
int bq769x0_balance_cell(int8_t cell);

#endif
