#ifndef __UNIT_ACMEASURE_H
#define __UNIT_ACMEASURE_H

#include "driver/i2c.h"

#define UNIT_ACMEASURE_ADDR 0x42
#define UNIT_ACMEASURE_VOLTAGE_STRING_REG 0x00
#define UNIT_ACMEASURE_CURRENT_STRING_REG 0x10
#define UNIT_ACMEASURE_POWER_STRING_REG 0x20
#define UNIT_ACMEASURE_APPARENT_POWER_STRING_REG 0x30
#define UNIT_ACMEASURE_POWER_FACTOR_STRING_REG 0x40
#define UNIT_ACMEASURE_KWH_STRING_REG 0x50
#define UNIT_ACMEASURE_VOLTAGE_REG 0x60
#define UNIT_ACMEASURE_CURRENT_REG 0x70
#define UNIT_ACMEASURE_POWER_REG 0x80
#define UNIT_ACMEASURE_APPARENT_POWER_REG 0x90
#define UNIT_ACMEASURE_POWER_FACTOR_REG 0xA0
#define UNIT_ACMEASURE_KWH_REG 0xB0
#define UNIT_ACMEASURE_VOLTAGE_FACTOR_REG 0xC0
#define UNIT_ACMEASURE_CURRENT_FACTOR_REG 0xD0
#define UNIT_ACMEASURE_SAVE_FACTOR_REG 0xE0
#define UNIT_ACMEASURE_GET_READY_REG 0xFC
#define JUMP_TO_BOOTLOADER_REG 0xFD
#define FIRMWARE_VERSION_REG 0xFE
#define I2C_ADDRESS_REG 0xFF

typedef struct {
  // I2C port information
  uint8_t addr;
  i2c_port_t port;
  uint8_t sda;
  uint8_t scl;
  uint8_t speed;
} UNIT_ACMEASURE;

void UNIT_ACMEASURE_init(void);
void UNIT_ACMEASURE_test(void);
uint32_t UNIT_ACMEASURE_getPowerWrapped(void);
uint8_t UNIT_ACMEASURE_getPowerFactorWrapped(void);
uint32_t UNIT_ACMEASURE_getApparentPowerWrapped(void);
float UNIT_ACMEASURE_getKWHWrapped(void);

#endif
