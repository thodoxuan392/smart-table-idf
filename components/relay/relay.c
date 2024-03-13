#include "relay.h"
#include <stdio.h>

#include <driver/gpio.h>

#define RELAY_IO_NUM GPIO_NUM_23

static gpio_config_t RELAY_io = {
    .mode = GPIO_MODE_INPUT_OUTPUT,       // MODE INPUT
    .pull_up_en = GPIO_PULLUP_DISABLE,    // Disable pull-up
    .pull_down_en = GPIO_PULLDOWN_ENABLE, // Disable pull-down resistor
    .intr_type = GPIO_INTR_DISABLE,       // Disable interrupt
    .pin_bit_mask = 1 << RELAY_IO_NUM,    // GPIO 23
};

void RELAY_init(void) { gpio_config(&RELAY_io); }
void RELAY_set(bool enable) { gpio_set_level(RELAY_IO_NUM, enable); }
bool RELAY_get() { return gpio_get_level(RELAY_IO_NUM) == 1; }