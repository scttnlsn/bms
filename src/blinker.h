#ifndef __BLINKER_H__
#define __BLINKER_H__

int blinker_init(char *device, int pin);
void blinker_on(void);
void blinker_off(void);
void blinker_flash(void);

#endif
