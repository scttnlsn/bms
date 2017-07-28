#ifndef __BQ769X0_H__
#define __BQ769X0_H__

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

typedef struct {
  uint16_t uvp; // mV
  uint16_t ovp; // mV
} bq769x0_config_t;

int bq769x0_init(void);
int bq769x0_boot(char *boot_port, int boot_pin);
int bq769x0_configure(bq769x0_config_t config);
int bq769x0_read_voltage(int cell_n, uint16_t *voltage);
int bq769x0_read_current(int16_t *current);
int bq769x0_read_status(uint8_t *status);
int bq769x0_enable_discharging(void);
int bq769x0_enable_charging(void);
int bq769x0_balance_cell(int8_t cell);

#endif
