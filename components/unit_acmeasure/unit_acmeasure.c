#include "unit_acmeasure.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include <driver/i2c.h>

#include <esp_log.h>

#define UNIT_ACMEASURE_TASK_NAME "ACMEASURE_TASK"
#define UNIT_ACMEASURE_TASK_STACK_SIZE 4096
#define UNIT_ACMEASURE_DELTA_MS_KWH 100
#define UNIT_ACMEASURE_MS_TO_H(ms) ((float)ms / 3600000.f)

static bool UNIT_ACMEASURE_begin(UNIT_ACMEASURE *unit);
static uint8_t UNIT_ACMEASURE_getFirmwareVersion(UNIT_ACMEASURE *unit);
static uint16_t UNIT_ACMEASURE_getVoltage(UNIT_ACMEASURE *unit);
static uint16_t UNIT_ACMEASURE_getCurrent(UNIT_ACMEASURE *unit);
static uint32_t UNIT_ACMEASURE_getPower(UNIT_ACMEASURE *unit);
static uint32_t UNIT_ACMEASURE_getApparentPower(UNIT_ACMEASURE *unit);
static uint8_t UNIT_ACMEASURE_getPowerFactor(UNIT_ACMEASURE *unit);
static uint32_t UNIT_ACMEASURE_getKWH(UNIT_ACMEASURE *unit);
static uint8_t UNIT_ACMEASURE_getReady(UNIT_ACMEASURE *unit);
static void UNIT_ACMEASURE_setKWH(UNIT_ACMEASURE *unit, uint32_t value);
static uint8_t UNIT_ACMEASURE_getVoltageFactor(UNIT_ACMEASURE *unit);
static uint8_t UNIT_ACMEASURE_getCurrentFactor(UNIT_ACMEASURE *unit);
static void UNIT_ACMEASURE_setVoltageFactor(UNIT_ACMEASURE *unit,
                                            uint8_t value);
static void UNIT_ACMEASURE_setCurrentFactor(UNIT_ACMEASURE *unit,
                                            uint8_t value);
static void UNIT_ACMEASURE_saveVoltageCurrentFactor(UNIT_ACMEASURE *unit);
static void UNIT_ACMEASURE_jumpBootloader(UNIT_ACMEASURE *unit);
static uint8_t UNIT_ACMEASURE_setI2CAddress(UNIT_ACMEASURE *unit, uint8_t addr);
static uint8_t UNIT_ACMEASURE_getI2CAddress(UNIT_ACMEASURE *unit);
static void UNIT_ACMEASURE_TASK_run(void *arg);
static void UNIT_ACMEASURE_TASK_timerCallback(TimerHandle_t xTimer);

static const UNIT_ACMEASURE UNIT_ACMEASURE_instance = {
    .addr = UNIT_ACMEASURE_ADDR,
    .port = I2C_NUM_0,
    .sda = GPIO_NUM_25,
    .scl = GPIO_NUM_21,
    .speed = 400000,
};

static StaticTask_t UNIT_ACMEASURE_TASK_xTaskBuffer;
static StackType_t UNIT_ACMEASURE_TASK_xStack[UNIT_ACMEASURE_TASK_STACK_SIZE];
static TaskHandle_t UNIT_ACMEASURE_TASK_taskHandle;
static TimerHandle_t UNIT_ACMEASURE_TIMER_timerHandle;
static StaticTimer_t UNIT_ACMEASURE_TIMER_buffer;

static const char *TAG = "UNIT_ACMEASURE";
static uint32_t UNIT_ACMEASURE_activePower = 0;
static uint32_t UNIT_ACMEASURE_apparentPower = 0;
static uint32_t UNIT_ACMEASURE_powerFactor = 0;
static float UNIT_ACMEASURE_kwh = 0;

void UNIT_ACMEASURE_init(void) {
  UNIT_ACMEASURE_begin(&UNIT_ACMEASURE_instance);
  UNIT_ACMEASURE_TASK_taskHandle = xTaskCreateStatic(
      UNIT_ACMEASURE_TASK_run, UNIT_ACMEASURE_TASK_NAME,
      UNIT_ACMEASURE_TASK_STACK_SIZE, NULL, 1, UNIT_ACMEASURE_TASK_xStack,
      &UNIT_ACMEASURE_TASK_xTaskBuffer);
  UNIT_ACMEASURE_TIMER_timerHandle = xTimerCreateStatic(
      "UnitAcMeasureTimer", pdMS_TO_TICKS(UNIT_ACMEASURE_DELTA_MS_KWH), pdTRUE,
      NULL, UNIT_ACMEASURE_TASK_timerCallback, &UNIT_ACMEASURE_TIMER_buffer);
  xTimerStart(UNIT_ACMEASURE_TIMER_timerHandle, 0);
}

void UNIT_ACMEASURE_test(void) {
  while (1) {
    // uint8_t firmwareVersion =
    //     UNIT_ACMEASURE_getFirmwareVersion(&UNIT_ACMEASURE_instance);
    // uint32_t power = UNIT_ACMEASURE_getPower()
    // ESP_LOGI(TAG, "Firmware Version: %x", firmwareVersion);
  }
}

uint32_t UNIT_ACMEASURE_getPowerWrapped(void) {
  return UNIT_ACMEASURE_activePower;
}
uint8_t UNIT_ACMEASURE_getPowerFactorWrapped(void) {
  return UNIT_ACMEASURE_powerFactor;
}
uint32_t UNIT_ACMEASURE_getApparentPowerWrapped(void) {
  return UNIT_ACMEASURE_apparentPower;
}

float UNIT_ACMEASURE_getKWHWrapped(void) { return UNIT_ACMEASURE_kwh; }

static void UNIT_ACMEASURE_TASK_run(void *arg) {
  while (1) {
    UNIT_ACMEASURE_activePower =
        UNIT_ACMEASURE_getPower(&UNIT_ACMEASURE_instance) / 100;
    UNIT_ACMEASURE_apparentPower =
        UNIT_ACMEASURE_getApparentPower(&UNIT_ACMEASURE_instance) / 100;
    UNIT_ACMEASURE_powerFactor =
        UNIT_ACMEASURE_getPowerFactor(&UNIT_ACMEASURE_instance);
    vTaskDelay(5);
  }
}

static void UNIT_ACMEASURE_TASK_timerCallback(TimerHandle_t xTimer) {
  UNIT_ACMEASURE_kwh += UNIT_ACMEASURE_MS_TO_H(UNIT_ACMEASURE_DELTA_MS_KWH) *
                        (float)UNIT_ACMEASURE_activePower / 1000;
  ESP_LOGI(TAG, "UNIT_ACMEASURE_kwh %f", UNIT_ACMEASURE_kwh);
}

static void UNIT_ACMEASURE_writeBytes(UNIT_ACMEASURE *unit, uint8_t addr,
                                      uint8_t reg, uint8_t *buffer,
                                      uint8_t length) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write(cmd, buffer, length, true);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(unit->port, cmd, portMAX_DELAY);
  i2c_cmd_link_delete(cmd);
}

static void UNIT_ACMEASURE_readBytes(UNIT_ACMEASURE *unit, uint8_t addr,
                                     uint8_t reg, uint8_t *buffer,
                                     uint8_t length) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, buffer, length, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  i2c_master_cmd_begin(unit->port, cmd, portMAX_DELAY);
  i2c_cmd_link_delete(cmd);
}

static bool UNIT_ACMEASURE_begin(UNIT_ACMEASURE *unit) {
  i2c_config_t conf = {0};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = unit->sda;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = unit->scl;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = unit->speed;
  i2c_param_config(unit->port, &conf);
  i2c_driver_install(unit->port, conf.mode, 0, 0, 0);
  i2c_set_timeout(unit->port, 10000);
  i2c_set_data_mode(unit->port, I2C_DATA_MODE_MSB_FIRST,
                    I2C_DATA_MODE_MSB_FIRST);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (unit->addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);
  esp_err_t error =
      i2c_master_cmd_begin(unit->port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (error == ESP_OK) {
    return true;
  } else {
    return false;
  }
}

static uint8_t UNIT_ACMEASURE_getFirmwareVersion(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, FIRMWARE_VERSION_REG, data, 1);
  return data[0];
}

static uint16_t UNIT_ACMEASURE_getVoltage(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, UNIT_ACMEASURE_VOLTAGE_REG, data,
                           2);
  uint16_t value = data[0] | (data[1] << 8);
  return value;
}
static uint16_t UNIT_ACMEASURE_getCurrent(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, UNIT_ACMEASURE_CURRENT_REG, data,
                           2);
  uint16_t value = data[0] | (data[1] << 8);
  return value;
}
static uint32_t UNIT_ACMEASURE_getPower(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, UNIT_ACMEASURE_POWER_REG, data, 4);
  uint32_t value = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
  return value;
}
static uint32_t UNIT_ACMEASURE_getApparentPower(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, UNIT_ACMEASURE_APPARENT_POWER_REG,
                           data, 4);
  uint32_t value = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
  return value;
}
static uint8_t UNIT_ACMEASURE_getPowerFactor(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, UNIT_ACMEASURE_POWER_FACTOR_REG,
                           data, 1);
  return data[0];
}
static uint32_t UNIT_ACMEASURE_getKWH(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, UNIT_ACMEASURE_KWH_REG, data, 4);
  uint32_t value = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
  return value;
}
static uint8_t UNIT_ACMEASURE_getReady(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, UNIT_ACMEASURE_GET_READY_REG, data,
                           1);
  return data[0];
}
static void UNIT_ACMEASURE_setKWH(UNIT_ACMEASURE *unit, uint32_t value) {
  UNIT_ACMEASURE_writeBytes(unit, unit->addr, UNIT_ACMEASURE_KWH_REG,
                            (uint8_t *)&value, 4);
}
static uint8_t UNIT_ACMEASURE_getVoltageFactor(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, UNIT_ACMEASURE_VOLTAGE_FACTOR_REG,
                           data, 1);
  return data[0];
}
static uint8_t UNIT_ACMEASURE_getCurrentFactor(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, UNIT_ACMEASURE_CURRENT_FACTOR_REG,
                           data, 1);
  return data[0];
}
static void UNIT_ACMEASURE_setVoltageFactor(UNIT_ACMEASURE *unit,
                                            uint8_t value) {
  UNIT_ACMEASURE_writeBytes(unit, unit->addr, UNIT_ACMEASURE_VOLTAGE_FACTOR_REG,
                            &value, 1);
}
static void UNIT_ACMEASURE_setCurrentFactor(UNIT_ACMEASURE *unit,
                                            uint8_t value) {
  UNIT_ACMEASURE_writeBytes(unit, unit->addr, UNIT_ACMEASURE_CURRENT_FACTOR_REG,
                            &value, 1);
}
static void UNIT_ACMEASURE_saveVoltageCurrentFactor(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, UNIT_ACMEASURE_SAVE_FACTOR_REG,
                           data, 1);
}
static void UNIT_ACMEASURE_jumpBootloader(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, JUMP_TO_BOOTLOADER_REG, data, 1);
}
static uint8_t UNIT_ACMEASURE_setI2CAddress(UNIT_ACMEASURE *unit,
                                            uint8_t addr) {
  UNIT_ACMEASURE_writeBytes(unit, unit->addr, I2C_ADDRESS_REG, &addr, 1);
  return addr;
}
static uint8_t UNIT_ACMEASURE_getI2CAddress(UNIT_ACMEASURE *unit) {
  uint8_t data[4];
  UNIT_ACMEASURE_readBytes(unit, unit->addr, I2C_ADDRESS_REG, data, 1);
  return data[0];
}
