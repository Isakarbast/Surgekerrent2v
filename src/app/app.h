#ifndef APP_H
#define APP_H

#include <stdbool.h>

#include "config.h"

void app_init(void);
void app_main_loop_iteration(void);
void app_clear_hw_faults(void);
/* True if the stored motor parameters are sufficient for the sensor mode. */
bool app_motor_ready(const app_config_t *cfg);

#endif
