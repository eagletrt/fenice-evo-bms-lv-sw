#ifndef RADIATOR_H
#define RADIATOR_H

#include <stdbool.h>

void radiator_init();
void radiator_set(float duty_cycle);
void radiator_set_auto_mode(bool mode);
bool radiator_get_auto_mode();
void radiator_auto_mode(float temp);

#endif // RADIATOR_H