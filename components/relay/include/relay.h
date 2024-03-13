#ifndef RELAY_H
#define RELAY_H

#include <stdbool.h>

void RELAY_init(void);
void RELAY_set(bool enable);
bool RELAY_get();

#endif // RELAY_H