#include <gpio.h>
#include <logging/sys_log.h>

#include "blinker.h"

static struct device *gpio;
static int gpio_pin;

int blinker_init(char *device, int pin) {
  gpio_pin = pin;

  gpio = device_get_binding(device);

  if (gpio == NULL) {
    SYS_LOG_ERR("could not find %s", device);
    return -EINVAL;
  }

  gpio_pin_configure(gpio, gpio_pin, GPIO_DIR_OUT);

  return 0;
}

void blinker_flash(void) {
  gpio_pin_write(gpio, gpio_pin, 1);
  k_sleep(100);
  gpio_pin_write(gpio, gpio_pin, 0);
}
