#ifndef __BQ769X0_H__
#define __BQ769X0_H__

#define BQ769X0_ADDR 0x08
#define BQ769X0_CFG_VALUE 0x19

// registers

#define BQ769X0_REG_CFG 0x0B

int bq769x0_init(void);
int bq769x0_boot(char *boot_port, int boot_pin);
int bq769x0_configure(void);

#endif
