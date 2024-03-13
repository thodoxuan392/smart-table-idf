#include "commandhandler.h"
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cJSON.h"

#include <relay.h>

static const char *TAG = "COMMANDHANDLER";

void COMMANDHANDLER_init() {
  // Nothing to do
}

bool COMMANDHANDLER_parse_command(char *payload, COMMAND_t *command) {
  cJSON *json = cJSON_Parse(payload);
  if (!json) {
    ESP_LOGE(TAG, "Cannot parse command from json string %s", payload);
    return false;
  }
  // Command
  cJSON *command_json =
      cJSON_GetObjectItemCaseSensitive(json, "relay"); // Required
  if (command_json && cJSON_IsBool(command_json)) {
    command->relay = cJSON_IsTrue(command_json);
  }
  cJSON_Delete(json);
  return true;
}

bool COMMANDHANDLER_handle_command(COMMAND_t *command) {
  RELAY_set(command->relay);
  return true;
}