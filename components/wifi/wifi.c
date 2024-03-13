#include "wifi.h"
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

enum { WIFI_STATE_INIT, WIFI_STATE_DISCONNECTED, WIFI_STATE_CONNECTED };

static const char *TAG = "WIFI";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t WIFI_event_group;
static bool connected = false;
// static esp_netif_t *esp_netif_sta;
static esp_netif_t *esp_netif_ap;
// static wifi_config_t _sta_config;
static wifi_config_t _ap_config = {
    .ap =
        {
            .channel = 1,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .ssid = "BIM_PLUG_001",
            .password = "ACLAB2023",
        },
};

static int s_retry_num = 0;

// Internal functions
static void WIFI_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
static bool WIFI_init_common(void);
static bool WIFI_init_softAp(void);
static bool WIFI_start(void);

bool WIFI_init(void) {
  WIFI_init_common();
  WIFI_init_softAp();
  WIFI_start();
  return true;
}

bool WIFI_is_connected(void) { return connected; }

/* Private function */

static void WIFI_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *event =
        (wifi_event_ap_staconnected_t *)event_data;
    ESP_LOGI(TAG, "Station " MACSTR " joined, AID=%d", MAC2STR(event->mac),
             event->aid);
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t *event =
        (wifi_event_ap_stadisconnected_t *)event_data;
    ESP_LOGI(TAG, "Station " MACSTR " left, AID=%d", MAC2STR(event->mac),
             event->aid);
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < 10) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      connected = false;
      xEventGroupSetBits(WIFI_event_group, WIFI_FAIL_BIT);
    }
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    connected = true;
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(WIFI_event_group, WIFI_CONNECTED_BIT);
  }
}

static bool WIFI_init_common(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  /* Initialize event group */
  WIFI_event_group = xEventGroupCreate();

  /* Register Event handler */
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &WIFI_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &WIFI_event_handler, NULL, NULL));

  /*Initialize WiFi */
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  return true;
}

/* Initialize soft AP */
static bool WIFI_init_softAp(void) {
  esp_netif_ap = esp_netif_create_default_wifi_ap();

  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &_ap_config));

  ESP_LOGI(TAG, "WIFI init softAp finished. SSID:%s password:%s",
           _ap_config.ap.ssid, _ap_config.ap.password);

  return true;
}

static bool WIFI_start(void) {
  /* Start WiFi */
  ESP_ERROR_CHECK(esp_wifi_start());
  return true;
}