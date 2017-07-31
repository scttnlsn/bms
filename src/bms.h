#ifndef __BMS_H__
#define __BMS_H__

int bms_init(void);
int bms_update(void);
uint16_t *bms_cell_voltages(void);
int16_t bms_current(void);
int32_t bms_charge(void);

#endif
