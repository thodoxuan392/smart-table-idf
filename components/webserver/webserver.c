#include "webserver.h"

#include <dnsserver.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_tls_crypto.h"
#include <esp_http_server.h>

#include "commandhandler.h"
#include "relay.h"
#include "unit_acmeasure.h"

#define UNAUTHORIZED_ERROR "Unauthorized error"
#define OUT_OF_MEMORY_ERROR "Out of memory"
#define OVERSIZE_ERROR "Oversize error"
#define INTERNAL_SERVER_ERROR "Internal Server Error"
#define MESSAGE_INVALID "Message Invalid"

// Handler
static esp_err_t WEBSERVER_index_handler(httpd_req_t *req);
static esp_err_t WEBSERVER_send_command_handler(httpd_req_t *req);
static esp_err_t WEBSERVER_get_status_handler(httpd_req_t *req);
static bool WEBSERVER_register(httpd_handle_t server, httpd_uri_t *uri_handle);

static const char *TAG = "WEBSERVER";
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

// Server handle
static httpd_handle_t server = NULL;

// URI description
static httpd_uri_t index_entry = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = WEBSERVER_index_handler,
};

static httpd_uri_t hotspot_detect_entry = {
    .uri = "/hotspot-detect.html",
    .method = HTTP_GET,
    .handler = WEBSERVER_index_handler,
};

static httpd_uri_t generate_204_entry = {
    .uri = "/generate_204",
    .method = HTTP_GET,
    .handler = WEBSERVER_index_handler,
};

static httpd_uri_t connecttest_entry = {
    .uri = "/connecttest.txt",
    .method = HTTP_GET,
    .handler = WEBSERVER_index_handler,
};

static httpd_uri_t send_command_entry = {
    .uri = "/command",
    .method = HTTP_POST,
    .handler = WEBSERVER_send_command_handler,
};

static httpd_uri_t get_status_entry = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = WEBSERVER_get_status_handler,
};

bool WEBSERVER_init(void) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.lru_purge_enable = true;

  // Start the httpd server
  ESP_LOGI(TAG, "Starting server on port: %d", config.server_port);
  if (httpd_start(&server, &config) != ESP_OK) {
    ESP_LOGE(TAG, "Error starting server!");
    return false;
  }
  // Set URI handlers
  ESP_LOGI(TAG, "Registering URI handlers");
  WEBSERVER_register(server, &index_entry);
  WEBSERVER_register(server, &hotspot_detect_entry);
  WEBSERVER_register(server, &generate_204_entry);
  WEBSERVER_register(server, &connecttest_entry);
  WEBSERVER_register(server, &send_command_entry);
  WEBSERVER_register(server, &get_status_entry);
  ESP_LOGI(TAG, "Registered URI all handlers");

  ESP_LOGI(TAG, "Starting DNS server ...");
  DNSSERVER_start();
  ESP_LOGI(TAG, "Started DNS server ...");
  return true;
}

// Helper
static bool WEBSERVER_register(httpd_handle_t server, httpd_uri_t *uri_handle) {
  return httpd_register_uri_handler(server, uri_handle) == ESP_OK;
}

// Handler
static esp_err_t WEBSERVER_index_handler(httpd_req_t *req) {
  // Send HTML index file
  char *buf = (char *)index_html_start;
  size_t buf_len = index_html_end - index_html_start;
  // End response
  httpd_resp_send(req, buf, buf_len);
  return ESP_OK;
}

static esp_err_t WEBSERVER_send_command_handler(httpd_req_t *req) {
  char *buf = (char *)calloc(1, COMMAND_BODY_MAX_LENGTH);
  if (!buf) {
    ESP_LOGE(TAG, "%s", OUT_OF_MEMORY_ERROR);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "Internal Server Error");
    return ESP_FAIL;
  }
  // Check if it over buffer
  if (req->content_len > COMMAND_BODY_MAX_LENGTH) {
    ESP_LOGE(TAG, "%s", OVERSIZE_ERROR);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, OVERSIZE_ERROR);
    return ESP_FAIL;
  }
  size_t buf_len = req->content_len;

  int ret = httpd_req_recv(req, buf, buf_len);
  if (ret <= 0) { /* 0 return value indicates connection closed */
    /* Check if timeout occurred */
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
      /* In case of timeout one can choose to retry calling
       * httpd_req_recv(), but to keep it simple, here we
       * respond with an HTTP 408 (Request Timeout) error */
      httpd_resp_send_408(req);
    }
    /* In case of error, returning ESP_FAIL will
     * ensure that the underlying socket is closed */
    free(buf);
    return ESP_FAIL;
  }

  // Parse config
  COMMAND_t command;
  if (!COMMANDHANDLER_parse_command(buf, &command)) {
    ESP_LOGE(TAG, "%s", MESSAGE_INVALID);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, MESSAGE_INVALID);
    free(buf);
    return ESP_FAIL;
  }
  // Set config
  if (!COMMANDHANDLER_handle_command(&command)) {
    ESP_LOGE(TAG, "%s", INTERNAL_SERVER_ERROR);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        INTERNAL_SERVER_ERROR);
    free(buf);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "Command executed successfully");
  /* Send a simple response */
  httpd_resp_send(req, NULL, 0);
  free(buf);
  return ESP_OK;
}

static esp_err_t WEBSERVER_get_status_handler(httpd_req_t *req) {
  char *buf = (char *)calloc(1, CONFIG_BODY_MAX_LENGTH);
  if (!buf) {
    ESP_LOGE(TAG, "%s", INTERNAL_SERVER_ERROR);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        INTERNAL_SERVER_ERROR);
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "Getting status ...");

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Connection", "keep-alive");

  size_t bufSize = snprintf(buf, CONFIG_BODY_MAX_LENGTH, "{\
          \"relay\": %d, \
          \"activePower\": %d, \
          \"apparentPower\": %d, \
          \"powerFactor\": %d, \
          \"kwh\": %f \
        }",
                            RELAY_get(), UNIT_ACMEASURE_getPowerWrapped(),
                            UNIT_ACMEASURE_getApparentPowerWrapped(),
                            UNIT_ACMEASURE_getPowerFactorWrapped(),
                            UNIT_ACMEASURE_getKWHWrapped());
  /* Send a simple response */
  httpd_resp_send(req, buf, bufSize);
  free(buf);
  return ESP_OK;
}
