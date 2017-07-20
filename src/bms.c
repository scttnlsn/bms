#include <logging/sys_log.h>

#include "bms.h"
#include "bq769x0.h"

int bms_init(void) {
  int ret;

  ret = bq769x0_init();
  if (ret < 0) {
    SYS_LOG_ERR("failed to initialize BQ769x0");
    return ret;
  }

  ret = bq769x0_boot("GPIO_0", 21);
  if (ret < 0) {
    SYS_LOG_ERR("failed to boot BQ769x0");
    return ret;
  }

  ret = bq769x0_configure();
  if (ret < 0) {
    SYS_LOG_ERR("failed to configure BQ769x0");
    return ret;
  }

  return 0;
}
