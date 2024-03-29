#include "esp_err.h"
#include "nvs_flash.h"
#include <stdio.h>
// App
#include "commandhandler.h"
#include "webserver.h"
#include "wifi.h"
// Hal
#include "relay.h"
#include "unit_acmeasure.h"

void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  RELAY_init();
  UNIT_ACMEASURE_init();
  // App init
  WIFI_init();
  WEBSERVER_init();
  COMMANDHANDLER_init();
}
