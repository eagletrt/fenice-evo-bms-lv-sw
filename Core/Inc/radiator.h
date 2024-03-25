#ifndef RADIATOR_H
#define RADIATOR_H

#include <stdbool.h>

void radiator_init();
void radiator_set_duty_cycle(float duty_cycle);
void radiator_set_auto_mode(bool mode);
float radiator_get_duty_cycle();
bool radiator_get_auto_mode();
void radiator_auto_mode(float temp);

#endif // RADIATOR_H