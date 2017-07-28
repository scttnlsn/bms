#ifndef __BMS_H__
#define __BMS_H__

int bms_init(void);
void bms_measure(void);
uint16_t *bms_cell_voltages(void);
int16_t bms_pack_current(void);

#endif
