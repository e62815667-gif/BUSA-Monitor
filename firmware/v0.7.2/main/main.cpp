#include <Arduino.h>
#include <esp_log.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_http_server.h>

// LAB-ONLY firmware: no GPIO setup, no relays, no heaters, no valves.
// No Wi-Fi credentials or network initialization. HTTP handlers are
// registered only if networking is added in a separately reviewed change.
static const char *TAG = "BUSA_MONITOR_ONLY";

static esp_err_t json_response(httpd_req_t *req, const char *json) {
  httpd_resp_set_type(req, "application/json; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_sendstr(req, json);
}
static esp_err_t capabilities(httpd_req_t *req) {
  return json_response(req, R"json({"device":"BUSA-1M-001","version":"v0.7.2-lab","api_read_only":true,"physical_outputs":false,"temperature_channels":4,"keyboard":"5x4 PCF8575","display":false})json");
}
static esp_err_t telemetry(httpd_req_t *req) {
  return json_response(req, R"json({"device":"BUSA-1M-001","version":"v0.7.2-lab","mode":"MONITOR_ONLY","outputs_enabled":false,"bridge_online":false,"keypad_online":false,"temperatures":[null,null,null,null],"channels":[{"valid":false,"age_ms":null},{"valid":false,"age_ms":null},{"valid":false,"age_ms":null},{"valid":false,"age_ms":null}],"last_key":null,"key_seq":0,"key_stream_id":"lab-boot-1","simulation":true,"k_states":{"K1":false,"K2":false,"K3":false,"K4":false}})json");
}
static esp_err_t keys(httpd_req_t *req) {
  return json_response(req, R"json({"stream_id":"lab-boot-1","events":[]})json");
}
// Not called by app_main: no network services are enabled in this build.
// Retained for API schema compilation and later isolated integration testing.
static void register_readonly_endpoints(httpd_handle_t server) {
  const httpd_uri_t endpoints[] = {
    { .uri="/api/capabilities", .method=HTTP_GET, .handler=capabilities, .user_ctx=nullptr },
    { .uri="/api/telemetry", .method=HTTP_GET, .handler=telemetry, .user_ctx=nullptr },
    { .uri="/api/keys", .method=HTTP_GET, .handler=keys, .user_ctx=nullptr }
  };
  for (const auto &endpoint : endpoints) {
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &endpoint));
  }
}
extern "C" void app_main(void) {
  initArduino();
  ESP_LOGW(TAG, "LAB ONLY: no sensors attached; no Wi-Fi; all physical outputs disabled");
  // The registration function is deliberately not called; preserve its symbol.
  (void)&register_readonly_endpoints;
  while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}
