#ifndef RADIATOR_H
#define RADIATOR_H

#include "primary_network.h"
#include <stdbool.h>

void radiator_init();
void radiator_set_duty_cycle(float duty_cycle);
void radiator_set_status(primary_lv_radiator_speed_status status);
float radiator_get_duty_cycle();
primary_lv_radiator_speed_status radiator_get_status();
bool radiator_is_auto();
void radiator_auto_mode(float temp);

#endif // RADIATOR_H