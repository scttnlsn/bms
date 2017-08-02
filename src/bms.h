#ifndef __BMS_H__
#define __BMS_H__

#define BMS_NUM_CELLS 4

int bms_init(void);
int bms_update(void);
int bms_cell_voltages(uint16_t *voltages);
int16_t bms_current(void);
int32_t bms_charge(void);

#endif
