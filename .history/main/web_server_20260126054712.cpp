#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "shared_state.h"
#include "web_server.h"

extern "C" {

static const char *TAG = "WEB_SERVER";

// 引用嵌入的文件
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t main_js_start[]    asm("_binary_main_js_start");
extern const uint8_t main_js_end[]      asm("_binary_main_js_end");
extern const uint8_t style_css_start[]  asm("_binary_style_css_start");
extern const uint8_t style_css_end[]    asm("_binary_style_css_end");

// GET /sensor - 返回 JSON 格式的传感器数据
static esp_err_t sensor_data_get_handler(httpd_req_t *req) {
    char json_response[256];
    snprintf(json_response, sizeof(json_response),
             "{\"acc\":[%.2f,%.2f,%.2f],\"gyro\":[%.2f,%.2f,%.2f],\"gps\":[%.6f,%.6f],\"sats\":%d}",
             g_sensor_data.acc_x, g_sensor_data.acc_y, g_sensor_data.acc_z,
             g_sensor_data.gyro_x, g_sensor_data.gyro_y, g_sensor_data.gyro_z,
             g_sensor_data.gps_lat, g_sensor_data.gps_lon, g_sensor_data.gps_sats);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, strlen(json_response));
    return ESP_OK;
}

static esp_err_t index_html_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

static esp_err_t main_js_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/javascript");
    httpd_resp_send(req, (const char *)main_js_start, main_js_end - main_js_start);
    return ESP_OK;
}

static esp_err_t style_css_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/css");
    httpd_resp_send(req, (const char *)style_css_start, style_css_end - style_css_start);
    return ESP_OK;
}

// 启动 Web 服务器
void start_web_server() {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = index_html_handler};
        httpd_uri_t js_uri = {.uri = "/main.js", .method = HTTP_GET, .handler = main_js_handler};
        httpd_uri_t css_uri = {.uri = "/style.css", .method = HTTP_GET, .handler = style_css_handler};
        httpd_uri_t sensor_uri = {.uri = "/api/sensor", .method = HTTP_GET, .handler = sensor_data_get_handler};

        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &js_uri);
        httpd_register_uri_handler(server, &css_uri);
        httpd_register_uri_handler(server, &sensor_uri);
        ESP_LOGI(TAG, "Web Server started at http://192.168.4.1");
    }
}

// 初始化 WiFi 接入点 (AP 模式)
void init_wifi_ap() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {};
    strcpy((char *)wifi_config.ap.ssid, "Manta_Robot_WiFi");
    wifi_config.ap.ssid_len = strlen("Manta_Robot_WiFi");
    wifi_config.ap.channel = 1;
    strcpy((char *)wifi_config.ap.password, "12345678");
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    if (strlen("12345678") == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP 'Manta_Robot_WiFi' started. Pass: 12345678");
}
